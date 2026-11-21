#include "main.h"

void FastLioLocalizationQnClass::publishCurrentPose(const PosePcd& pose_pcd)
{
  if (!m_localized || !localization_validation::validTransform(pose_pcd.pose_corrected_eig)) return;
  geometry_msgs::PoseStamped pose = poseEigToPoseStamped(pose_pcd.pose_corrected_eig, m_map_frame);
  pose.header.stamp = pose_pcd.stamp;
  m_realtime_pose_pub.publish(pose);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
  transform.setRotation(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                      pose.pose.orientation.z, pose.pose.orientation.w));
  m_broadcaster.sendTransform(tf::StampedTransform(transform, pose.header.stamp, m_map_frame, "robot"));
  sensor_msgs::PointCloud2 cloud = pclToPclRos(transformPcd(pose_pcd.pcd, pose_pcd.pose_corrected_eig), m_map_frame);
  cloud.header.stamp = pose.header.stamp;
  m_corrected_current_pcd_pub.publish(cloud);
}

void FastLioLocalizationQnClass::odomPcdCallback(const nav_msgs::OdometryConstPtr& odom_msg,
                                               const sensor_msgs::PointCloud2ConstPtr& pcd_msg)
{
  try
  {
    m_current_frame = PosePcd(*odom_msg, *pcd_msg, m_current_keyframe_idx);
  }
  catch (const std::exception& error)
  {
    ROS_ERROR_STREAM_THROTTLE(5.0, "Rejecting invalid live odometry/cloud: " << error.what());
    return;
  }
  m_current_frame.pose_corrected_eig = m_last_TF * m_current_frame.pose_eig;

  if (!m_init)
  {
    m_last_keyframe = m_current_frame;
    m_not_processed_keyframe = m_current_frame;
    updateVisVars(m_current_frame);
    ++m_current_keyframe_idx;
    m_init = true;
  }
  else if (!m_localized)
  {
    // Keep retrying initialization with fresh scans even while the robot is stationary.
    // Reuse the initial visualization index until a global match is accepted.
    m_current_frame.idx = m_last_keyframe.idx;
    m_last_keyframe = m_current_frame;
    m_not_processed_keyframe = m_current_frame;
    const std::size_t index = static_cast<std::size_t>(m_current_frame.idx);
    if (index < m_odoms.points.size() && index < m_odom_path.poses.size())
    {
      m_odoms.points[index] = pcl::PointXYZ(m_current_frame.pose_eig(0, 3),
                                          m_current_frame.pose_eig(1, 3),
                                          m_current_frame.pose_eig(2, 3));
      m_odom_path.poses[index] = poseEigToPoseStamped(m_current_frame.pose_eig, m_map_frame);
    }
  }
  else if (checkIfKeyframe(m_current_frame, m_last_keyframe))
  {
    m_last_keyframe = m_current_frame;
    m_not_processed_keyframe = m_current_frame;
    updateVisVars(m_current_frame);
    ++m_current_keyframe_idx;
  }

  publishCurrentPose(m_current_frame);
}

void FastLioLocalizationQnClass::matchingTimerFunc(const ros::TimerEvent& event)
{
  (void)event;
  // Publish the latched map independently of keyframe processing.
  if (m_saved_map_vis_switch && m_saved_map_pub.getNumSubscribers() > 0)
  {
    m_saved_map_pub.publish(pclToPclRos(m_saved_map_pcd, m_map_frame));
    m_saved_map_vis_switch = false;
  }
  if (!m_saved_map_vis_switch && m_saved_map_pub.getNumSubscribers() == 0)
    m_saved_map_vis_switch = true;

  if (!m_init || m_not_processed_keyframe.processed) return;
  const auto start = high_resolution_clock::now();
  PosePcd pending = m_not_processed_keyframe;
  m_not_processed_keyframe.processed = true;
  // Refresh the correction in case this keyframe was queued before the last match.
  pending.pose_corrected_eig = m_last_TF * pending.pose_eig;
  if (m_saved_map.empty())
  {
    ROS_FATAL("Localization map unexpectedly became empty; stopping node");
    ros::shutdown();
    return;
  }

  const int closest = getClosestKeyframeIdx(pending, m_saved_map);
  bool converged = false;
  double score = std::numeric_limits<double>::infinity();
  Eigen::Matrix4d correction = Eigen::Matrix4d::Identity();
  try
  {
    if (closest >= 0)
    {
      correction = m_enable_quatro ?
          coarseToFineKeyToKey(pending, closest, m_saved_map, converged, score) :
          icpKeyToSubkeys(pending, closest, m_saved_map, converged, score);
    }
  }
  catch (const std::exception& error)
  {
    ROS_ERROR_STREAM_THROTTLE(5.0, "Map registration failed: " << error.what());
    converged = false;
  }

  if (converged && localization_validation::validTransform(correction))
  {
    const Eigen::Matrix4d next_tf = correction * m_last_TF;
    const Eigen::Matrix4d corrected_pose = next_tf * pending.pose_eig;
    if (!localization_validation::validTransform(next_tf) ||
        !localization_validation::validTransform(corrected_pose))
    {
      ROS_ERROR("Rejecting map registration because the resulting pose/transform is invalid");
      return;
    }
    const std::size_t index = pending.idx < 0 ? std::numeric_limits<std::size_t>::max() :
                                              static_cast<std::size_t>(pending.idx);
    if (index >= m_corrected_odoms.points.size() || index >= m_corrected_path.poses.size() ||
        index >= m_odoms.points.size())
    {
      ROS_FATAL("Keyframe/visualization indices are inconsistent; stopping node");
      ros::shutdown();
      return;
    }
    m_last_TF = next_tf;
    m_corrected_odoms.points[index] = pcl::PointXYZ(corrected_pose(0, 3), corrected_pose(1, 3), corrected_pose(2, 3));
    m_corrected_path.poses[index] = poseEigToPoseStamped(corrected_pose, m_map_frame);
    m_match_xyz_pairs.push_back({m_corrected_odoms.points[index], m_odoms.points[index]});
    m_map_match_pub.publish(getMatchMarker(m_match_xyz_pairs));
    if (!m_localized)
      ROS_INFO("Initial map registration accepted (score %.6f). Publishing global localization.", score);
    m_localized = true;
    m_current_frame.pose_corrected_eig = m_last_TF * m_current_frame.pose_eig;
    publishCurrentPose(m_current_frame);
  }
  else if (!m_localized)
  {
    ROS_WARN_THROTTLE(5.0, "No accepted initial map match. Global pose/TF are withheld; check starting position, map, extrinsics, timestamps and point-cloud overlap.");
  }
  else
  {
    ROS_WARN_THROTTLE(5.0, "Map match rejected; continuing odometry with the last accepted map correction.");
  }

  if (m_localized)
  {
    m_corrected_odom_pub.publish(pclToPclRos(m_corrected_odoms, m_map_frame));
    m_corrected_path_pub.publish(m_corrected_path);
  }
  m_odom_pub.publish(pclToPclRos(m_odoms, m_map_frame));
  m_path_pub.publish(m_odom_path);
  const auto finish = high_resolution_clock::now();
  ROS_INFO("Map registration + visualization: %.1f ms, score %.6f",
           duration_cast<microseconds>(finish - start).count() / 1e3, score);
}
