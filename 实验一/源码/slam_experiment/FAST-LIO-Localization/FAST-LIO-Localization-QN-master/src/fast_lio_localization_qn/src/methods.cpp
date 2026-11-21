#include "main.h"


void FastLioLocalizationQnClass::updateVisVars(const PosePcd& pose_pcd_in)
{
  m_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  m_corrected_odoms.points.emplace_back(pose_pcd_in.pose_corrected_eig(0, 3), pose_pcd_in.pose_corrected_eig(1, 3), pose_pcd_in.pose_corrected_eig(2, 3));
  m_odom_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_eig, m_map_frame));
  m_corrected_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig, m_map_frame));
  return;
}

visualization_msgs::Marker FastLioLocalizationQnClass::getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& match_xyz_pairs)
{
  visualization_msgs::Marker edges_; edges_.type = 5u;
  edges_.scale.x = 0.2f; edges_.header.frame_id = m_map_frame; edges_.pose.orientation.w = 1.0f;
  edges_.color.r = 1.0f; edges_.color.g = 1.0f; edges_.color.b = 1.0f; edges_.color.a = 1.0f;
  {
    for (int i = 0; i < match_xyz_pairs.size(); ++i)
    {
      geometry_msgs::Point p_, p2_;
      p_.x = match_xyz_pairs[i].first.x; p_.y = match_xyz_pairs[i].first.y; p_.z = match_xyz_pairs[i].first.z;
      p2_.x = match_xyz_pairs[i].second.x; p2_.y = match_xyz_pairs[i].second.y; p2_.z = match_xyz_pairs[i].second.z;
      edges_.points.push_back(p_);
      edges_.points.push_back(p2_);
    }
  }
  return edges_;
}

void FastLioLocalizationQnClass::voxelizePcd(pcl::VoxelGrid<PointType>& voxelgrid, pcl::PointCloud<PointType>& pcd_in)
{
  if (pcd_in.empty()) return;
  for (const auto& point : pcd_in.points)
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::runtime_error("refusing to voxelize non-finite point coordinates");
  pcl::PointCloud<PointType>::Ptr before_(new pcl::PointCloud<PointType>);
  *before_ = pcd_in;
  voxelgrid.setInputCloud(before_);
  voxelgrid.filter(pcd_in);
  return;
}

bool FastLioLocalizationQnClass::checkIfKeyframe(const PosePcd& pose_pcd_in, const PosePcd& latest_pose_pcd)
{
  // A map correction must not itself count as physical motion.
  return m_keyframe_thr < (latest_pose_pcd.pose_eig.block<3, 1>(0, 3) - pose_pcd_in.pose_eig.block<3, 1>(0, 3)).norm();
}

int FastLioLocalizationQnClass::getClosestKeyframeIdx(const PosePcd& current_keyframe, const std::vector<PosePcdReduced>& saved_map)
{
  double shortest_distance_ = m_match_det_radi*3.0;
  int closest_idx_ = -1;
  if (!localization_validation::validTransform(current_keyframe.pose_corrected_eig)) return -1;
  for (std::size_t idx = 0; idx < saved_map.size(); ++idx)
  {
    //check if potential matchee: close enough in distance, then get the closest one
    double tmp_dist_ = (saved_map[idx].pose_eig.block<3, 1>(0, 3) - current_keyframe.pose_corrected_eig.block<3, 1>(0, 3)).norm();
    if (m_match_det_radi > tmp_dist_)
    {
      if (tmp_dist_ < shortest_distance_)
      {
        shortest_distance_ = tmp_dist_;
        closest_idx_ = static_cast<int>(idx);
      }
    }
  }
  return closest_idx_;
}

Eigen::Matrix4d FastLioLocalizationQnClass::icpKeyToSubkeys(const PosePcd& current_keyframe, const int& closest_idx, const std::vector<PosePcdReduced>& keyframes, bool& if_converged, double& score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  score = std::numeric_limits<double>::infinity();
  if (closest_idx < 0 || static_cast<std::size_t>(closest_idx) >= keyframes.size() || current_keyframe.pcd.empty())
  {
    ROS_WARN_THROTTLE(5.0, "Skipping ICP: invalid map keyframe index or empty current cloud");
    return output_tf_;
  }
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType> dst_raw_, src_raw_;
  src_raw_ = transformPcd(current_keyframe.pcd, current_keyframe.pose_corrected_eig);
  const std::int64_t first = std::max<std::int64_t>(0, static_cast<std::int64_t>(closest_idx) - m_sub_key_num);
  const std::int64_t last = std::min<std::int64_t>(static_cast<std::int64_t>(keyframes.size()) - 1,
                                                static_cast<std::int64_t>(closest_idx) + m_sub_key_num);
  for (std::int64_t i = first; i <= last; ++i)
  {
    dst_raw_ += transformPcd(keyframes[static_cast<std::size_t>(i)].pcd, keyframes[static_cast<std::size_t>(i)].pose_eig);
  }
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // Nano-GICP allocates k covariance neighbors without initializing absent ones.
  if (src_raw_.size() < static_cast<std::size_t>(m_min_registration_points) ||
      dst_raw_.size() < static_cast<std::size_t>(m_min_registration_points))
  {
    ROS_WARN_THROTTLE(5.0, "Skipping ICP: voxelized source/target has fewer points than /nano_gicp/correspondences_number");
    return output_tf_;
  }
  // then match with Nano-GICP
  pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
  *dst_ = dst_raw_;
  *src_ = src_raw_;
  pcl::PointCloud<PointType> aligned_;
  m_nano_gicp.setInputSource(src_);
  m_nano_gicp.calculateSourceCovariances();
  m_nano_gicp.setInputTarget(dst_);
  m_nano_gicp.calculateTargetCovariances();
  m_nano_gicp.align(aligned_);
  // vis for debug
  m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
  m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
  m_debug_fine_aligned_pub.publish(pclToPclRos(aligned_, m_map_frame));
  // handle results
  score = m_nano_gicp.getFitnessScore();
  const Eigen::Matrix4d candidate = m_nano_gicp.getFinalTransformation().cast<double>();
  if(m_nano_gicp.hasConverged() && std::isfinite(score) && score < m_icp_score_thr &&
     localization_validation::validTransform(candidate) && !aligned_.empty())
  {
    if_converged = true;
    output_tf_ = candidate;
  }
  return output_tf_;
}

Eigen::Matrix4d FastLioLocalizationQnClass::coarseToFineKeyToKey(const PosePcd& current_keyframe, const int& closest_idx, const std::vector<PosePcdReduced>& keyframes, bool& if_converged, double& score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  score = std::numeric_limits<double>::infinity();
  if (!m_quatro_handler || closest_idx < 0 || static_cast<std::size_t>(closest_idx) >= keyframes.size() || current_keyframe.pcd.empty())
  {
    ROS_WARN_THROTTLE(5.0, "Skipping Quatro: unavailable handler, invalid map index, or empty current cloud");
    return output_tf_;
  }
  // Prepare the keyframes
  pcl::PointCloud<PointType> dst_raw_, src_raw_;
  src_raw_ = transformPcd(current_keyframe.pcd, current_keyframe.pose_corrected_eig);
  dst_raw_ = transformPcd(keyframes[closest_idx].pcd, keyframes[closest_idx].pose_eig); //Note: Quatro should work on scan-to-scan (keyframe-to-keyframe), not keyframe-to-merged-many-keyframes
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  if (src_raw_.size() < static_cast<std::size_t>(m_min_registration_points) ||
      dst_raw_.size() < static_cast<std::size_t>(m_min_registration_points))
  {
    ROS_WARN_THROTTLE(5.0, "Skipping Quatro/ICP: voxelized source/target has too few points");
    return output_tf_;
  }
  // then perform Quatro
  Eigen::Matrix4d quatro_tf_ = m_quatro_handler->align(src_raw_, dst_raw_, if_converged);
  if (!if_converged || !localization_validation::validTransform(quatro_tf_))
  {
    if_converged = false;
    return output_tf_;
  }
  else //if valid,
  {
    // coarse align with the result of Quatro
    pcl::PointCloud<PointType> src_coarse_aligned_ = transformPcd(src_raw_, quatro_tf_);
    // then match with Nano-GICP
    pcl::PointCloud<PointType> fine_aligned_;
    pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
    *dst_ = dst_raw_;
    *src_ = src_coarse_aligned_;
    m_nano_gicp.setInputSource(src_);
    m_nano_gicp.calculateSourceCovariances();
    m_nano_gicp.setInputTarget(dst_);
    m_nano_gicp.calculateTargetCovariances();
    m_nano_gicp.align(fine_aligned_);
    // handle results
    score = m_nano_gicp.getFitnessScore();
    const Eigen::Matrix4d icp_tf_ = m_nano_gicp.getFinalTransformation().cast<double>();
    if(m_nano_gicp.hasConverged() && std::isfinite(score) && score < m_icp_score_thr &&
       localization_validation::validTransform(icp_tf_) && !fine_aligned_.empty())
    {
      if_converged = true;
      output_tf_ = icp_tf_ * quatro_tf_; // IMPORTANT: take care of the order
    }
    else if_converged = false;
    // vis for debug
    m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
    m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
    m_debug_coarse_aligned_pub.publish(pclToPclRos(src_coarse_aligned_, m_map_frame));
    m_debug_fine_aligned_pub.publish(pclToPclRos(fine_aligned_, m_map_frame));
  }

  return output_tf_;
}

void FastLioLocalizationQnClass::loadMap(const std::string& saved_map_path)
{
  try
  {
    rosbag::Bag bag_;
    bag_.open(saved_map_path, rosbag::bagmode::Read);
    rosbag::View cloud_view(bag_, rosbag::TopicQuery("/keyframe_pcd"));
    rosbag::View pose_view(bag_, rosbag::TopicQuery("/keyframe_pose"));
    std::vector<sensor_msgs::PointCloud2> clouds;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (const rosbag::MessageInstance& message : cloud_view)
    {
      const auto cloud = message.instantiate<sensor_msgs::PointCloud2>();
      if (!cloud)
        throw std::runtime_error("/keyframe_pcd must contain sensor_msgs/PointCloud2 messages");
      clouds.push_back(*cloud);
    }
    for (const rosbag::MessageInstance& message : pose_view)
    {
      const auto pose = message.instantiate<geometry_msgs::PoseStamped>();
      if (!pose)
        throw std::runtime_error("/keyframe_pose must contain geometry_msgs/PoseStamped messages");
      poses.push_back(*pose);
    }
    if (clouds.empty() || poses.empty())
      throw std::runtime_error("map bag has no /keyframe_pcd or /keyframe_pose messages; a scans.pcd or raw sensor bag is not a localization map");
    if (clouds.size() != poses.size())
      throw std::runtime_error("map topic counts differ: /keyframe_pcd=" + std::to_string(clouds.size()) +
                               ", /keyframe_pose=" + std::to_string(poses.size()));
    if (clouds.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
      throw std::runtime_error("map has too many keyframes for integer keyframe indices");

    // Keep the member map empty until every pair has passed validation.
    std::vector<PosePcdReduced> keyframes;
    pcl::PointCloud<PointType> global_cloud;
    std::string source_frame;
    bool empty_pose_frame = false;
    keyframes.reserve(poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
      try
      {
        if (!clouds[i].header.stamp.isZero() && !poses[i].header.stamp.isZero() &&
            clouds[i].header.stamp != poses[i].header.stamp)
          throw std::runtime_error("paired cloud/pose header stamps differ");
        if (poses[i].header.frame_id.empty()) empty_pose_frame = true;
        else if (source_frame.empty()) source_frame = poses[i].header.frame_id;
        else if (poses[i].header.frame_id != source_frame)
          throw std::runtime_error("mixed pose frames '" + source_frame + "' and '" + poses[i].header.frame_id + "' in one map bag");
        keyframes.emplace_back(poses[i], clouds[i], static_cast<int>(i));
        const auto transformed = transformPcd(keyframes.back().pcd, keyframes.back().pose_eig);
        for (const auto& point : transformed.points)
          if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            throw std::runtime_error("global point coordinates became non-finite after applying pose");
        global_cloud += transformed;
      }
      catch (const std::exception& error)
      {
        throw std::runtime_error("keyframe " + std::to_string(i) + ": " + error.what());
      }
    }
    voxelizePcd(m_voxelgrid, global_cloud);
    if (global_cloud.empty()) throw std::runtime_error("map point cloud is empty after voxel filtering");
    bag_.close();
    m_saved_map.swap(keyframes);
    m_saved_map_pcd.swap(global_cloud);
    if (empty_pose_frame) ROS_WARN("Some map pose frame IDs are empty; preserving legacy map coordinates");
    if (!source_frame.empty() && source_frame != m_map_frame)
      ROS_INFO_STREAM("Loaded coordinates from frame '" << source_frame << "' as '" << m_map_frame
                      << "' (same mapping origin; no coordinate transformation applied)");
    ROS_INFO_STREAM("Loaded map '" << saved_map_path << "': " << m_saved_map.size()
                    << " keyframes, " << m_saved_map_pcd.size() << " visualization points");
  }
  catch (const std::exception& error)
  {
    throw std::runtime_error("Failed to load localization map '" + saved_map_path + "': " + error.what());
  }
}
