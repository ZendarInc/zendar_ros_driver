#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <zendar/api/api.h>

namespace zen {
class ZendarDriverNode {
public:
  ZendarDriverNode(
    const ros::NodeHandle& node,
    const std::string& url,
    int argc,
    char* argv[]
  );
  ~ZendarDriverNode();
  void Run();
private:
  ros::NodeHandle node;
  std::unique_ptr<image_transport::ImageTransport> image_transport;
  std::array<float, 3> FindMinMedianMax(const std::vector<uint32_t>& sorted_array);
  float ScaleMagToImage(const uint32_t& magnitude, const std::array<float, 3>& min_median_max, const float& max_value);
  std::vector<uint32_t> DownsampleArray(const uint32_t* data, const size_t& size, const size_t& factor);
  void PublishPointCloud(
    std::unordered_map<std::string, ros::Publisher>* cloud_publishers,
    std::unordered_map<std::string, ros::Publisher>* cloud_metadata_publishers);
  void PublishImage(
    std::unordered_map<std::string, image_transport::Publisher>* image_publishers);
  void PublishLogs(ros::Publisher& log_publisher);
  void PublishPoseQuality(ros::Publisher& pose_quality_publisher);
  void PublishPose(ros::Publisher& pose_publisher);
  geometry_msgs::PoseStamped ConvertToPoseStamped(const zpb::tracker::message::TrackerState& cloud_data);
  sensor_msgs::PointCloud2 ConvertToPointCloud2(const zpb::tracker::message::TrackerState& cloud_data);
};
}  // namespace zen
