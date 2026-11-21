#include "main.h"


PosePcd::PosePcd(const nav_msgs::Odometry& odom_in, const sensor_msgs::PointCloud2& pcd_in, const int& idx_in)
{
  if (odom_in.header.stamp.isZero() || pcd_in.header.stamp.isZero())
    throw std::runtime_error("live odometry/cloud timestamps must be non-zero");
  if (odom_in.header.stamp != pcd_in.header.stamp)
    throw std::runtime_error("live odometry/cloud timestamps must match exactly");
  if (odom_in.header.frame_id.empty() || odom_in.header.frame_id != pcd_in.header.frame_id)
    throw std::runtime_error("live odometry/cloud must use the same non-empty world frame (FAST-LIO: camera_init)");
  pose_eig = localization_validation::poseMatrix(odom_in.pose.pose);
  pose_corrected_eig = pose_eig;
  const pcl::PointCloud<PointType> tmp_pcd_ = localization_validation::pointCloud(pcd_in);
  pcd = transformPcd(tmp_pcd_, pose_eig.inverse()); // FAST-LIO /cloud_registered is in the odometry world frame; retain body coordinates.
  for (const auto& point : pcd.points)
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      throw std::runtime_error("live cloud coordinates became non-finite after applying odometry pose");
  idx = idx_in;
  stamp = odom_in.header.stamp;
}
PosePcdReduced::PosePcdReduced(const geometry_msgs::PoseStamped& pose_in, const sensor_msgs::PointCloud2& pcd_in, const int& idx_in)
{
  pose_eig = localization_validation::poseMatrix(pose_in.pose);
  pcd = localization_validation::pointCloud(pcd_in);
  idx = idx_in;
}

FastLioLocalizationQnClass::FastLioLocalizationQnClass(const ros::NodeHandle& n_private) : m_nh(n_private)
{
  ////// ROS params
  // temp vars, only used in constructor
  std::string saved_map_path_;
  double map_match_hz_;
  double quatro_gicp_vox_res_;
  int nano_thread_number_, nano_correspondences_number_, nano_max_iter_;
  int nano_ransac_max_iter_, quatro_max_iter_, quatro_max_corres_;
  double transformation_epsilon_, euclidean_fitness_epsilon_, ransac_outlier_rejection_threshold_;
  double fpfh_normal_radius_, fpfh_radius_, noise_bound_, rot_gnc_factor_, rot_cost_diff_thr_;
  double quatro_distance_threshold_;
  bool estimat_scale_, use_optimized_matching_;
  // get params
  /* basic */
  m_nh.param<std::string>("/basic/map_frame", m_map_frame, "map");
  m_nh.param<std::string>("/basic/saved_map", saved_map_path_, "");
  m_nh.param<double>("/basic/map_match_hz", map_match_hz_, 1.0);
  m_nh.param<double>("/match/quatro_nano_gicp_voxel_resolution", quatro_gicp_vox_res_, 0.3);
  /* keyframe */
  m_nh.param<double>("/keyframe/keyframe_threshold", m_keyframe_thr, 1.0);
  m_nh.param<int>("/keyframe/subkeyframes_number", m_sub_key_num, 5);
  /* match */
  m_nh.param<double>("/match/match_detection_radius", m_match_det_radi, 15.0);
  /* nano */
  m_nh.param<int>("/nano_gicp/thread_number", nano_thread_number_, 0);
  m_nh.param<double>("/nano_gicp/icp_score_threshold", m_icp_score_thr, 10.0);
  m_nh.param<int>("/nano_gicp/correspondences_number", nano_correspondences_number_, 15);
  m_nh.param<int>("/nano_gicp/max_iter", nano_max_iter_, 32);
  m_nh.param<double>("/nano_gicp/transformation_epsilon", transformation_epsilon_, 0.01);
  m_nh.param<double>("/nano_gicp/euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 0.01);
  m_nh.param<int>("/nano_gicp/ransac/max_iter", nano_ransac_max_iter_, 5);
  m_nh.param<double>("/nano_gicp/ransac/outlier_rejection_threshold", ransac_outlier_rejection_threshold_, 1.0);
  /* quatro */
  m_nh.param<bool>("/quatro/enable", m_enable_quatro, false);
  m_nh.param<bool>("/quatro/optimize_matching", use_optimized_matching_, true);
  m_nh.param<double>("/quatro/distance_threshold", quatro_distance_threshold_, 30.0);
  m_nh.param<int>("/quatro/max_correspondences", quatro_max_corres_, 200);
  m_nh.param<double>("/quatro/fpfh_normal_radius", fpfh_normal_radius_, 0.02);
  m_nh.param<double>("/quatro/fpfh_radius", fpfh_radius_, 0.04);
  m_nh.param<bool>("/quatro/estimating_scale", estimat_scale_, false);
  m_nh.param<double>("/quatro/noise_bound", noise_bound_, 0.25);
  m_nh.param<double>("/quatro/rotation/gnc_factor", rot_gnc_factor_, 1.4);
  m_nh.param<double>("/quatro/rotation/rot_cost_diff_threshold", rot_cost_diff_thr_, 0.25);
  m_nh.param<int>("/quatro/rotation/num_max_iter", quatro_max_iter_, 50);

  const auto require_positive = [](double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0)
      throw std::runtime_error(std::string("Parameter ") + name + " must be finite and > 0");
  };
  if (saved_map_path_.empty())
    throw std::runtime_error("Missing /basic/saved_map: provide a map .bag containing /keyframe_pcd and /keyframe_pose");
  if (m_map_frame.empty()) throw std::runtime_error("/basic/map_frame must not be empty");
  require_positive(map_match_hz_, "/basic/map_match_hz");
  require_positive(quatro_gicp_vox_res_, "/match/quatro_nano_gicp_voxel_resolution");
  require_positive(m_keyframe_thr, "/keyframe/keyframe_threshold");
  require_positive(m_match_det_radi, "/match/match_detection_radius");
  require_positive(m_icp_score_thr, "/nano_gicp/icp_score_threshold");
  require_positive(transformation_epsilon_, "/nano_gicp/transformation_epsilon");
  require_positive(euclidean_fitness_epsilon_, "/nano_gicp/euclidean_fitness_epsilon");
  require_positive(ransac_outlier_rejection_threshold_, "/nano_gicp/ransac/outlier_rejection_threshold");
  if (m_sub_key_num < 0 || nano_thread_number_ < 0 || nano_correspondences_number_ < 3 || nano_max_iter_ < 1 || nano_ransac_max_iter_ < 0)
    throw std::runtime_error("Invalid keyframe/Nano-GICP integer parameter (subkeyframes >= 0, threads >= 0, correspondences >= 3, max_iter >= 1, RANSAC iterations >= 0)");
  m_min_registration_points = nano_correspondences_number_;
  if (m_enable_quatro)
  {
    require_positive(fpfh_normal_radius_, "/quatro/fpfh_normal_radius");
    require_positive(fpfh_radius_, "/quatro/fpfh_radius");
    require_positive(noise_bound_, "/quatro/noise_bound");
    require_positive(rot_gnc_factor_, "/quatro/rotation/gnc_factor");
    if (rot_gnc_factor_ <= 1.0)
      throw std::runtime_error("/quatro/rotation/gnc_factor must be > 1");
    require_positive(rot_cost_diff_thr_, "/quatro/rotation/rot_cost_diff_threshold");
    require_positive(quatro_distance_threshold_, "/quatro/distance_threshold");
    if (quatro_max_corres_ < 3 || quatro_max_iter_ < 1)
      throw std::runtime_error("Quatro requires max_correspondences >= 3 and num_max_iter >= 1");
  }

  ////// Matching init
  // Voxel init
  m_voxelgrid.setLeafSize(quatro_gicp_vox_res_, quatro_gicp_vox_res_, quatro_gicp_vox_res_);
  // nano_gicp init
  m_nano_gicp.setMaxCorrespondenceDistance(m_match_det_radi*2.0);
  m_nano_gicp.setNumThreads(nano_thread_number_);
  m_nano_gicp.setCorrespondenceRandomness(nano_correspondences_number_);
  m_nano_gicp.setMaximumIterations(nano_max_iter_);
  m_nano_gicp.setTransformationEpsilon(transformation_epsilon_);
  m_nano_gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
  m_nano_gicp.setRANSACIterations(nano_ransac_max_iter_);
  m_nano_gicp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold_);
  // quatro init
  if (m_enable_quatro) m_quatro_handler = std::make_shared<quatro<PointType>>(fpfh_normal_radius_, fpfh_radius_, noise_bound_, rot_gnc_factor_, rot_cost_diff_thr_,
                                                        quatro_max_iter_, estimat_scale_, use_optimized_matching_, quatro_distance_threshold_, quatro_max_corres_);
  // Load map
  loadMap(saved_map_path_);

  ////// ROS things
  m_odom_path.header.frame_id = m_map_frame;
  m_corrected_path.header.frame_id = m_map_frame;
  // publishers
  m_odom_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/ori_odom", 10, true);
  m_path_pub = m_nh.advertise<nav_msgs::Path>("/ori_path", 10, true);
  m_corrected_odom_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/corrected_odom", 10, true);
  m_corrected_path_pub = m_nh.advertise<nav_msgs::Path>("/corrected_path", 10, true);
  m_corrected_current_pcd_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/corrected_current_pcd", 10, true);
  m_map_match_pub = m_nh.advertise<visualization_msgs::Marker>("/map_match", 10, true);
  m_realtime_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/pose_stamped", 10);
  m_saved_map_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/saved_map", 10, true);
  m_debug_src_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/src", 10);
  m_debug_dst_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/dst", 10);
  m_debug_coarse_aligned_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/coarse_aligned_quatro", 10);
  m_debug_fine_aligned_pub = m_nh.advertise<sensor_msgs::PointCloud2>("/fine_aligned_nano_gicp", 10);
  // subscribers
  m_sub_odom = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(m_nh, "/Odometry", 10);
  m_sub_pcd = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(m_nh, "/cloud_registered", 10);
  m_sub_odom_pcd_sync = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *m_sub_odom, *m_sub_pcd);
  m_sub_odom_pcd_sync->registerCallback(boost::bind(&FastLioLocalizationQnClass::odomPcdCallback, this, _1, _2));
  // Timers at the end
  m_match_timer = m_nh.createTimer(ros::Duration(1/map_match_hz_), &FastLioLocalizationQnClass::matchingTimerFunc, this);
  
  ROS_WARN("Map loaded. Waiting for the first successful map registration before publishing /pose_stamped or map->robot TF.");
}
