#define CLOUD_STREAM_TEST
#include "robot-cloud-stream.cpp"
#include <cassert>
#include <limits>

int main() {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "camera_init"; cloud.header.stamp = ros::Time(100, 0);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(24514);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (size_t i = 0; i < 24514; ++i, ++x, ++y, ++z) { *x = i * 0.01f; *y = 2; *z = 3; }
  Batch batch;
  assert(append(cloud, batch)); assert(batch.points.size() == 24514); assert(batch.frames == 1);
  cloud.header.stamp = ros::Time(100, 100000000);
  assert(append(cloud, batch)); assert(batch.points.size() == 49028); assert(batch.frames == 2);
  assert(batch.sourcePoints == 49028); assert(batch.points.back()[0] == 24513 * 0.01f);
  cloud.header.frame_id = "other_world";
  assert(append(cloud, batch)); assert(batch.points.size() == 24514); assert(batch.frames == 1);
  cloud.header.stamp = ros::Time(10, 0);
  assert(append(cloud, batch)); assert(batch.points.size() == 24514); assert(batch.firstStamp == 10);
  batch.clear(); sensor_msgs::PointCloud2Iterator<float> invalid(cloud, "x"); *invalid = std::numeric_limits<float>::quiet_NaN();
  assert(append(cloud, batch)); assert(batch.points.size() == 24513); assert(batch.sourcePoints == 24514);
  std::ostringstream output; auto* original = std::cout.rdbuf(output.rdbuf());
  emit(batch, "raw", 1); std::cout.rdbuf(original);
  assert(output.str().find("\"sourcePoints\":24514") != std::string::npos);
  assert(output.str().find("nan") == std::string::npos);
  std::cout << "PASS: complete frames, aggregation, frame/clock reset, finite XYZ, serialization\n";
}
