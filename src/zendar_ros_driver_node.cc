#include "zendar_ros_driver/zendar_driver_node.h"

#include <glog/logging.h>
#include <utility>
#include <vector>
#include <pcl/common/distances.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <rosgraph_msgs/Log.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <cv_bridge/cv_bridge.h>

#include "zendar_ros_driver/zendar_point.h"

namespace zen {
namespace {
constexpr int loop_rate_Hz = 100;
}  // namespace


ZendarDriverNode::ZendarDriverNode(
  const ros::NodeHandle& node,
  const std::string& url,
  int argc,
  char* argv[]
)
  : node(node)
{
  api::ZenApi::Init(&argc, &argv);

  auto default_telem_ports = api::ZenApi::TelemPortOptions();
  api::ZenApi::Connect(url, default_telem_ports);

  auto default_data_ports = api::ZenApi::DataPortOptions();
  api::ZenApi::Bind(default_data_ports);

  api::ZenApi::SubscribeImages();
  api::ZenApi::SubscribeTrackerStates();
  api::ZenApi::SubscribeTracklogs();
  api::ZenApi::SubscribeHousekeepingReports();
  api::ZenApi::SubscribeLogMessages();

  image_transport
      = std::make_unique<image_transport::ImageTransport>(node);
}

ZendarDriverNode::~ZendarDriverNode() {
  // graceful exit once done
  api::ZenApi::UnsubscribeTrackerStates();
  api::ZenApi::UnsubscribeImages();
  api::ZenApi::UnsubscribeLogMessages();
  api::ZenApi::UnsubscribeTracklogs();
  api::ZenApi::UnsubscribeHousekeepingReports();
  api::ZenApi::Release();
  api::ZenApi::Disconnect();
}

void ZendarDriverNode::PublishImage(
    std::unordered_map<std::string, image_transport::Publisher>* image_publishers) {
  auto image = api::ZenApi::NextImage();
  std::unordered_map<std::string, image_transport::Publisher>::iterator image_publisher = image_publishers->find(image->meta().serial());
  if (image_publisher == image_publishers->end()) {
    image_publishers->emplace(std::make_pair(image->meta().serial(), image_transport->advertise("image_" + image->meta().serial(), 1)));
  }
}

void ZendarDriverNode::PublishPointCloud(
    std::unordered_map<std::string, ros::Publisher>* cloud_publishers,
    std::unordered_map<std::string, ros::Publisher>* cloud_metadata_publishers) {
  auto cloud = api::ZenApi::NextTrackerState();
  std::unordered_map<std::string, ros::Publisher>::iterator cloud_publisher = cloud_publishers->find(cloud->meta().serial());
  if (cloud_publisher == cloud_publishers->end()) {
    cloud_publishers->emplace(std::make_pair(cloud->meta().serial(), node.advertise<sensor_msgs::PointCloud2>("points_" + cloud->meta().serial(), 1)));
    cloud_metadata_publishers->emplace(std::make_pair(cloud->meta().serial(), node.advertise<geometry_msgs::PoseStamped>("points_metadata_" + cloud->meta().serial(), 1)));
  }
  cloud_publishers->at(cloud->meta().serial()).publish(ConvertToPointCloud2(*cloud));
  cloud_metadata_publishers->at(cloud->meta().serial()).publish(ConvertToPoseStamped(*cloud));
}

geometry_msgs::PoseStamped ZendarDriverNode::ConvertToPoseStamped(const zpb::tracker::message::TrackerState& cloud_data) {
  const auto& attitude_proto = cloud_data.meta().attitude();
  const auto& position_proto = cloud_data.meta().position();

  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header.seq = (uint32_t)(cloud_data.meta().frame_id());
  pose_stamped_msg.header.stamp = ros::Time(cloud_data.meta().timestamp());
  pose_stamped_msg.header.frame_id = "ECEF";

  pose_stamped_msg.pose.position.x = position_proto.x();
  pose_stamped_msg.pose.position.y = position_proto.y();
  pose_stamped_msg.pose.position.z = position_proto.z();

  pose_stamped_msg.pose.orientation.w = attitude_proto.w();
  pose_stamped_msg.pose.orientation.x = attitude_proto.x();
  pose_stamped_msg.pose.orientation.y = attitude_proto.y();
  pose_stamped_msg.pose.orientation.z = attitude_proto.z();

  return pose_stamped_msg;
}


void ZendarDriverNode::PublishLogs(ros::Publisher& log_publisher) {
  auto next_log = api::ZenApi::NextLogMessage();
  rosgraph_msgs::Log log_msg;
  // severity is set using the glog severity levels
  switch (next_log->severity()) {
  // GLOG_INFO
  case 0:
    log_msg.level = 2;
    break;
  // GLOG_WARNING
  case 1:
    log_msg.level = 4;
    break;
  case 2:
  // GLOG_ERROR
    log_msg.level = 8;
    break;
  // GLOG_FATAL
  case 3:
    log_msg.level = 16;
    break;
  // Set the log level to FATAL for other levels, since they should all be
  // more severe.
  default:
    log_msg.level = 16;
  }
  log_msg.name = next_log->base_filename();
  log_msg.msg = next_log->message();
  log_msg.file = next_log->full_filename();
  log_msg.line = next_log->file_line();

  log_msg.header.stamp = ros::Time(next_log->timestamp());
  log_publisher.publish(log_msg);
}

void ZendarDriverNode::PublishPoseQuality(ros::Publisher& pose_quality_publisher) {
  auto next_tracklog = api::ZenApi::NextTracklog();
  const auto& next_quality = next_tracklog->quality();
  diagnostic_msgs::DiagnosticArray diagnostics_array;
  diagnostic_msgs::DiagnosticStatus gps_status;
  gps_status.name = "GPS Status";
  gps_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  switch (next_quality.gps_status()) {
  case (zpb::GpsFix::NO_FIX):
    gps_status.message = "No Fix";
    break;
  case (zpb::GpsFix::TIME_ONLY):
    gps_status.message = "Time Only";
    break;
  case (zpb::GpsFix::FIX_2D):
    gps_status.message = "Fix 2D";
    break;
  case (zpb::GpsFix::FIX_3D):
    gps_status.message = "Fix 3D";
    break;
  case (zpb::GpsFix::SBAS):
    gps_status.message = "SBAS";
    break;
  default:
    gps_status.message = "Unknown GPS Status";
  }

  diagnostic_msgs::KeyValue num_sats;
  num_sats.key = "Number of satellites";
  num_sats.value = std::to_string(next_quality.satellite_count());
  gps_status.values.push_back(num_sats);
  diagnostics_array.status.push_back(gps_status);

  diagnostic_msgs::DiagnosticStatus ins_status;
  ins_status.name = "INS Status";
  ins_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  switch (next_quality.ins_status()) {
  case (zpb::data::InsMode::NOT_TRACKING):
    ins_status.message = "Not Tracking";
    break;
  case (zpb::data::InsMode::ALIGNING):
    ins_status.message = "Aligning";
    break;
  case (zpb::data::InsMode::TRACKING):
    ins_status.message = "Tracking";
    break;
  default:
    ins_status.message = "Unknown INS Status";
  }
  diagnostics_array.status.push_back(ins_status);
  diagnostics_array.header.stamp = ros::Time(next_tracklog->timestamp());
  pose_quality_publisher.publish(diagnostics_array);
}

void ZendarDriverNode::PublishPose(ros::Publisher& pose_publisher) {

  // we use tracker state proto instead of tracklog is because
  // tracker state has position and orientation aligned with
  // the point cloud.
  auto next_tracklog = api::ZenApi::NextTracklog();
  const auto& next_attitude = next_tracklog->attitude();
  const auto& next_position = next_tracklog->position();

  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header.stamp = ros::Time(next_tracklog->timestamp());
  pose_stamped_msg.header.frame_id = "ECEF";

  pose_stamped_msg.pose.position.x = next_position.x();
  pose_stamped_msg.pose.position.y = next_position.y();
  pose_stamped_msg.pose.position.z = next_position.z();

  pose_stamped_msg.pose.orientation.w = next_attitude.w();
  pose_stamped_msg.pose.orientation.x = next_attitude.x();
  pose_stamped_msg.pose.orientation.y = next_attitude.y();
  pose_stamped_msg.pose.orientation.z = next_attitude.z();

  pose_publisher.publish(pose_stamped_msg);
}

void ZendarDriverNode::Run() {
  ros::Rate loop_rate(loop_rate_Hz);
  std::unordered_map<std::string, image_transport::Publisher> image_publishers;
  std::unordered_map<std::string, ros::Publisher> cloud_publishers;
  std::unordered_map<std::string, ros::Publisher> cloud_metadata_publishers;
  std::unordered_map<std::string, double> cloud_timestamps;
  std::unordered_map<std::string, double> image_timestamps;
  ros::Publisher log_publisher
    = node.advertise<rosgraph_msgs::Log>("/zpu_logs", 5);
  ros::Publisher pose_quality_publisher
    = node.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 5);
  ros::Publisher pose_publisher
    = node.advertise<geometry_msgs::PoseStamped>("/pose", 5);
  while (node.ok()) {
    PublishPointCloud(&cloud_publishers, &cloud_metadata_publishers);
    PublishImage(&image_publishers);
    PublishLogs(log_publisher);
    PublishPoseQuality(pose_quality_publisher);
    PublishPose(pose_publisher);
    loop_rate.sleep();
  }
}

sensor_msgs::PointCloud2 ConvertToPointCloud2(const zpb::tracker::message::TrackerState& cloud_data) {
  pcl::PointCloud<ZendarPoint> pointcloud;
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.seq = (uint32_t)(cloud_data.meta().frame_id());
  cloud_msg.header.frame_id = cloud_data.meta().serial();
  cloud_msg.header.stamp = ros::Time(cloud_data.meta().timestamp());

  // Fill the PointCloud2
  for (auto point_data : cloud_data.detection()) {
    ZendarPoint point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    point.x_ecef = point_data.position().x();
    point.y_ecef = point_data.position().y();
    point.z_ecef = point_data.position().z();
    point.mag = point_data.magnitude();
    point.az_var = point_data.azimuth_variance();
    point.el_var = point_data.elevation_variance();
    point.r = point_data.range();
    point.rad_vel = point_data.range_velocity();
    point.az = point_data.azimuth();
    point.el = point_data.elevation();
    point.doa_snr_db = point_data.doa_snr_db();
    point.rd_mean_snr_db = point_data.rd_mean_snr_db();
    pointcloud.push_back(point);
  }
  return cloud_msg;
}
}  // namespace zen
