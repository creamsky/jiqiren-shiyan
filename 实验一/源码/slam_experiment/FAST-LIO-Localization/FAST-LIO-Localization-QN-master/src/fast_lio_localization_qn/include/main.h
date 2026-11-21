#ifndef FAST_LIO_LOCALIZATION_QN_MAIN_H
#define FAST_LIO_LOCALIZATION_QN_MAIN_H

///// coded headers
#include "utilities.h"
///// common headers
#include <time.h>
#include <math.h>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <iterator>
#include <memory>
///// ROS
#include <ros/ros.h>
#include <rosbag/bag.h> // load map
#include <rosbag/view.h> // load map
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler
#include <tf/transform_datatypes.h> // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h> // tf <-> eigen
#include <tf/transform_broadcaster.h> // broadcaster
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/common/transforms.h> //transformPointCloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/filters/voxel_grid.h> //voxelgrid
///// Nano-GICP
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
///// Quatro
#include <quatro/quatro_module.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)


using namespace std::chrono;
using PointType = pcl::PointXYZI;
typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

// Validate external messages before TF/PCL can assert or use invalid input.
namespace localization_validation
{
inline Eigen::Matrix4d poseMatrix(const geometry_msgs::Pose& pose)
{
  const auto& p = pose.position;
  const auto& q = pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    throw std::runtime_error("pose contains NaN or infinity");
  Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < 1e-12)
    throw std::runtime_error("pose quaternion has zero/invalid norm");
  quaternion.normalize();
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = Eigen::Vector3d(p.x, p.y, p.z);
  return matrix;
}

inline bool validTransform(const Eigen::Matrix4d& matrix)
{
  if (!matrix.allFinite()) return false;
  const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
  return (matrix.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() < 1e-6 &&
         (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-3 &&
         std::abs(rotation.determinant() - 1.0) < 1e-3;
}

inline pcl::PointCloud<PointType> pointCloud(const sensor_msgs::PointCloud2& message)
{
  const std::uint64_t point_count = static_cast<std::uint64_t>(message.width) * message.height;
  if (point_count == 0 || message.point_step == 0 || message.data.empty())
    throw std::runtime_error("PointCloud2 is empty");
  const std::uint64_t row_bytes = static_cast<std::uint64_t>(message.width) * message.point_step;
  const std::uint64_t required_bytes = static_cast<std::uint64_t>(message.row_step) *
                                    (message.height - 1) + row_bytes;
  if (message.row_step < row_bytes || message.data.size() < required_bytes)
    throw std::runtime_error("PointCloud2 has invalid row_step/data length");
  if (message.is_bigendian)
    throw std::runtime_error("big-endian PointCloud2 is unsupported; use FLOAT32 little-endian XYZ");
  bool has_intensity = false;
  for (const std::string name : {"x", "y", "z", "intensity"})
  {
    const auto field = std::find_if(message.fields.begin(), message.fields.end(),
        [&name](const sensor_msgs::PointField& value) { return value.name == name; });
    if (field == message.fields.end())
    {
      if (name != "intensity") throw std::runtime_error("PointCloud2 is missing field " + name);
      continue;
    }
    if (field->datatype != sensor_msgs::PointField::FLOAT32 || field->count != 1 ||
        static_cast<std::uint64_t>(field->offset) + sizeof(float) > message.point_step)
      throw std::runtime_error("PointCloud2 field " + name + " must be a valid scalar FLOAT32");
    if (name == "intensity") has_intensity = true;
  }
  pcl::PointCloud<PointType> output;
  if (has_intensity) pcl::fromROSMsg(message, output);
  else
  {
    pcl::PointCloud<pcl::PointXYZ> xyz;
    pcl::fromROSMsg(message, xyz);
    output.reserve(xyz.size());
    for (const auto& point : xyz)
    {
      PointType converted;
      converted.x = point.x; converted.y = point.y; converted.z = point.z; converted.intensity = 0.0f;
      output.push_back(converted);
    }
  }
  const auto invalid = std::remove_if(output.points.begin(), output.points.end(),
      [](const PointType& point) {
        return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
      });
  const std::size_t invalid_count = std::distance(invalid, output.points.end());
  output.points.erase(invalid, output.points.end());
  if (output.empty()) throw std::runtime_error("PointCloud2 has no finite XYZ points");
  if (invalid_count > 0) ROS_WARN_STREAM("Discarded " << invalid_count << " non-finite XYZ points");
  for (auto& point : output.points)
    if (!std::isfinite(point.intensity)) point.intensity = 0.0f;
  output.width = static_cast<std::uint32_t>(output.size());
  output.height = 1;
  output.is_dense = true;
  return output;
}
} // namespace localization_validation


////////////////////////////////////////////////////////////////////////////////////////////////////
struct PosePcd
{
  pcl::PointCloud<PointType> pcd;
  Eigen::Matrix4d pose_eig = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_corrected_eig = Eigen::Matrix4d::Identity();
  int idx = -1;
  ros::Time stamp;
  bool processed = false;
  PosePcd(){};
  PosePcd(const nav_msgs::Odometry& odom_in, const sensor_msgs::PointCloud2& pcd_in, const int& idx_in);
};
struct PosePcdReduced
{
  pcl::PointCloud<PointType> pcd;
  Eigen::Matrix4d pose_eig = Eigen::Matrix4d::Identity();
  int idx = -1;
  PosePcdReduced(){};
  PosePcdReduced(const geometry_msgs::PoseStamped& pose_in, const sensor_msgs::PointCloud2& pcd_in, const int& idx_in);
};
////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioLocalizationQnClass
{
  private:
    ///// basic params
    std::string m_map_frame;
    ///// shared data - odom and pcd
    std::mutex m_keyframes_mutex, m_vis_mutex;
    bool m_init=false;
    bool m_localized=false;
    int m_current_keyframe_idx = 0;
    PosePcd m_current_frame, m_last_keyframe, m_not_processed_keyframe;
    std::vector<PosePcdReduced> m_saved_map;
    Eigen::Matrix4d m_last_TF = Eigen::Matrix4d::Identity();
    ///// map match
    pcl::VoxelGrid<PointType> m_voxelgrid;
    nano_gicp::NanoGICP<PointType, PointType> m_nano_gicp;
    std::shared_ptr<quatro<PointType>> m_quatro_handler = nullptr;
    bool m_enable_quatro = false;
    double m_keyframe_thr;
    double m_icp_score_thr;
    double m_match_det_radi;
    int m_sub_key_num;
    int m_min_registration_points;
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> m_match_xyz_pairs; //for vis
    ///// visualize
    tf::TransformBroadcaster m_broadcaster;
    pcl::PointCloud<pcl::PointXYZ> m_odoms, m_corrected_odoms;
    nav_msgs::Path m_odom_path, m_corrected_path;
    pcl::PointCloud<PointType> m_saved_map_pcd;
    bool m_saved_map_vis_switch = true;
    ///// ros
    ros::NodeHandle m_nh;
    ros::Publisher m_corrected_odom_pub, m_corrected_path_pub, m_odom_pub, m_path_pub;
    ros::Publisher m_corrected_current_pcd_pub, m_realtime_pose_pub, m_map_match_pub;
    ros::Publisher m_saved_map_pub;
    ros::Publisher m_debug_src_pub, m_debug_dst_pub, m_debug_coarse_aligned_pub, m_debug_fine_aligned_pub;
    ros::Timer m_match_timer;
    // odom, pcd sync subscriber
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> m_sub_odom_pcd_sync = nullptr;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> m_sub_odom = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> m_sub_pcd = nullptr;

    ///// functions
  public:
    FastLioLocalizationQnClass(const ros::NodeHandle& n_private);
    ~FastLioLocalizationQnClass(){};
  private:
    //methods
    void updateVisVars(const PosePcd& pose_pcd_in);
    void voxelizePcd(pcl::VoxelGrid<PointType>& voxelgrid, pcl::PointCloud<PointType>& pcd_in);
    bool checkIfKeyframe(const PosePcd& pose_pcd_in, const PosePcd& latest_pose_pcd);
    int getClosestKeyframeIdx(const PosePcd& current_keyframe, const std::vector<PosePcdReduced>& saved_map);
    Eigen::Matrix4d icpKeyToSubkeys(const PosePcd& current_keyframe, const int& closest_idx, const std::vector<PosePcdReduced>& keyframes, bool& if_converged, double& score);
    Eigen::Matrix4d coarseToFineKeyToKey(const PosePcd& current_keyframe, const int& closest_idx, const std::vector<PosePcdReduced>& keyframes, bool& if_converged, double& score);
    visualization_msgs::Marker getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& match_xyz_pairs);
    void loadMap(const std::string& saved_map_path);
    void publishCurrentPose(const PosePcd& pose_pcd);
    //cb
    void odomPcdCallback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2ConstPtr& pcd_msg);
    void matchingTimerFunc(const ros::TimerEvent& event);
};



#endif
