#include "main.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_lio_localization_qn_node");
  ros::NodeHandle nh_private("~");

  try
  {
    FastLioLocalizationQnClass localization(nh_private);
    // These callbacks share map correction, keyframe and visualization state.
    // Serial ROS callbacks remove the races; Nano-GICP may still parallelize internally.
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("Localization startup/runtime failed: " << error.what());
    return 1;
  }
 
  return 0;
}
