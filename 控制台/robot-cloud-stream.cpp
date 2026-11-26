// The high-volume sources are subscribed on demand and delivered as bounded previews.
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

struct Batch {
  std::vector<std::array<float, 3>> points;
  std::string frame;
  double stamp = 0, firstStamp = 0;
  size_t frames = 0, sourcePoints = 0;
  void clear() { points.clear(); frame.clear(); stamp = firstStamp = 0; frames = sourcePoints = 0; }
};

std::string quoted(const std::string& value) {
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c >= 32 && c < 127) out << c;
    else out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c) << std::dec;
  }
  out << '"'; return out.str();
}

bool append(const sensor_msgs::PointCloud2& msg, Batch& batch) {
  try {
    // Same frame only; a restarted ROS clock starts a new aggregation window.
    if (batch.frame != msg.header.frame_id || msg.header.stamp.toSec() <= batch.stamp) batch.clear();
    const size_t count = size_t(msg.width) * msg.height;
    if (batch.points.size() + count > 1000000) {
      batch.clear(); std::cerr << "Point cloud exceeds the viewer memory budget; window discarded\n"; return false;
    }
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) batch.points.push_back({*x, *y, *z});
    }
    batch.frame = msg.header.frame_id;
    batch.stamp = msg.header.stamp.toSec();
    if (!batch.frames) batch.firstStamp = batch.stamp;
    ++batch.frames; batch.sourcePoints += count;
    return true;
  } catch (const std::exception& error) {
    batch.clear(); std::cerr << "Invalid point cloud: " << error.what() << '\n'; return false;
  }
}

void emit(const Batch& batch, const char* source, unsigned long long sequence) {
  std::ostringstream out;
  out << "{\"type\":\"cloud\",\"source\":" << quoted(source)
      << ",\"frame\":" << quoted(batch.frame) << ",\"registered\":" << (std::string(source) == "aggregate" ? "true" : "false")
      << ",\"stamp\":" << std::setprecision(16) << batch.stamp << ",\"sequence\":" << sequence
      << ",\"frames\":" << batch.frames << ",\"sourcePoints\":" << batch.sourcePoints
      << ",\"windowMs\":" << (std::string(source) == "aggregate" ? 1000 : 0)
      << ",\"spanMs\":" << (batch.stamp - batch.firstStamp) * 1000 << ",\"points\":[";
  // Rotate a bounded preview sample while retaining original input counts.
  out << std::setprecision(9);
  bool first = true;
  const size_t stride=std::max<size_t>(1,(batch.points.size()+4095)/4096);
  for (size_t i=sequence%stride;i<batch.points.size();i+=stride) {
    const auto& p=batch.points[i];
    if (!first) out << ','; first = false;
    out << '[' << p[0] << ',' << p[1] << ',' << p[2] << ']';
  }
  out << "]}\n"; std::cout << out.str();
}

#ifndef CLOUD_STREAM_TEST
int main(int argc, char** argv) {
  ros::init(argc, argv, "webCloudViewer", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  const std::string source=argc==2?argv[1]:"";
  if(source!="raw"&&source!="aggregate")return 2;
  Batch aggregate, raw;
  // Retain the latest raw message, parsing it only at the 1 Hz delivery rate.
  sensor_msgs::PointCloud2ConstPtr latestRaw;
  unsigned long long sequence = 0;
  auto lastCloudAt = ros::WallTime::now();
  ros::Subscriber worldSub,rawSub;
  if(source=="aggregate")worldSub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 5,
      [&](const sensor_msgs::PointCloud2ConstPtr& msg) { lastCloudAt = ros::WallTime::now(); append(*msg, aggregate); }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  if(source=="raw")rawSub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1,
      [&](const sensor_msgs::PointCloud2ConstPtr& msg) { lastCloudAt = ros::WallTime::now(); latestRaw = msg; }, ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
  auto timer = nh.createWallTimer(ros::WallDuration(1), [&](const ros::WallTimerEvent&) {
    // An experiment may restart roscore. Reconnect instead of retaining orphaned subscriptions.
    if ((ros::WallTime::now() - lastCloudAt).toSec() > 12) {
      std::cerr << "No cloud input; reconnecting ROS subscriptions\n"; ros::shutdown(); return;
    }
    ++sequence;
    if (aggregate.frames) { emit(aggregate, "aggregate", sequence); aggregate.clear(); }
    if (latestRaw) { raw.clear(); if (append(*latestRaw, raw)) emit(raw, "raw", sequence); latestRaw.reset(); }
    std::cout.flush();
  });
  ros::spin();
}
#endif
