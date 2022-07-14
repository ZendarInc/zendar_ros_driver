#include "zendar_ros_driver/zendar_driver_node.h"
#include "zendar_ros_driver/zendar_point.h"
#include "zendar_ros_driver/range_markers.h"
#include "zendar_ros_driver/ego_vehicle.h"
#include "zendar_ros_driver/tracks.h"

// #include <ros/ros.h>
// #include <diagnostic_msgs/DiagnosticArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <image_transport/image_transport.h>
// #include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

#include <glog/logging.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>

using namespace zen;
using namespace zen::api;


/// \namespace -----------------------------------------------------------------
namespace {

constexpr int LOOP_RATE_HZ                 = 100;

constexpr size_t LOG_MSG_QUEUE             = 200;

constexpr size_t IMAGE_DOWNSAMPLING_FACTOR = 5;
constexpr float IM_DYN_RANGE_MAX           = std::pow(10.0, (60.0 / 20.0)); ///< 60dB dynamic range
constexpr float IM_DYN_RANGE_MIN           = std::pow(10.0, (55.0 / 20.0));  ///< 55dB dynamic range

///< max value of image set to 99% of atan's output
const float ATAN_SCALE_FACTOR          = std::tan(0.99 * M_PI_2);

// Parameters for occupancy grid display
const int OCCUPIED_VALUE = 0;
const int UNOCCUPIED_VALUE = 100;

struct ImageNormal
{
  ImageNormal(
    const zpb::data::Image& source,
    std::size_t downsample_factor,
    float min,
    float max
  ) {
    auto source_data = source.cartesian().data().data();
    auto source_cols = source.cartesian().data().cols();
    auto source_rows = source.cartesian().data().rows();

    ROS_ASSERT(source_data.size() == sizeof(uint32_t) * source_cols * source_rows);
    const auto* aligned_data = reinterpret_cast<const uint32_t*>(source_data.data());

    std::size_t downsampled_size = source_data.size() / (sizeof(uint32_t) * downsample_factor);
    std::vector<uint32_t> downsampled_data(downsampled_size);
    for (size_t ndx = 0; ndx < downsampled_size; ++ndx) {
      downsampled_data[ndx] = aligned_data[ndx * downsample_factor];
    }

    downsampled_data.erase(
      std::remove(downsampled_data.begin(), downsampled_data.end(), 0),
      downsampled_data.end()
    );
    std::sort(downsampled_data.begin(), downsampled_data.end());

    this->min = downsampled_data.front();
    this->med = downsampled_data.at(downsampled_data.size() / 2);
    this->max = downsampled_data.back();
    this->limit = this->max / this->med;
    this->limit =
      std::min(IM_DYN_RANGE_MAX, std::max(IM_DYN_RANGE_MIN, this->limit));
  }

  float min;
  float med;
  float max;
  float limit;
};

class ImageProcessor
{
public:
  ImageProcessor(const zpb::data::Image& source)
    : data(source.cartesian().data().data())
    , cols(source.cartesian().data().cols())
    , rows(source.cartesian().data().rows())
  {
    ROS_ASSERT(this->data.size() == sizeof(uint32_t) * this->cols * this->rows);
  }

  ImageProcessor&
  Scale(const ImageNormal& normal, float scale_factor)
  {
    using PixelMap = const uint32_t (*)[this->cols][this->rows];
    const auto pixel_map = *reinterpret_cast<PixelMap>(this->data.data());

    cv::Mat scaled_frame(this->rows, this->cols, CV_8UC1);
    for (int col = 0; col < this->cols; ++col) {
      for (int row = 0; row < this->rows; ++row) {
        float pixel = pixel_map[col][row];
        float scaled_pixel = pixel / normal.med;

        if (scaled_pixel < 1.0) {
          scaled_pixel = 0.0;
        }
        else {
          // subtracting 1 to center 0dB SNR to 0
          scaled_pixel =
            std::atan((scaled_pixel - 1.0) * scale_factor / (normal.limit - 1.0));
        }

        scaled_frame.at<uint8_t>(row, col) = scaled_pixel * 255 / M_PI_2;
      }
    }

    this->frame = std::move(scaled_frame);

    return *this;
  }

  ImageProcessor&
  Color()
  {
    cv::Mat colored_frame(this->cols, this->rows, CV_8UC3);
    cv::applyColorMap(this->frame, colored_frame, cv::COLORMAP_INFERNO);
    cv::flip(colored_frame, colored_frame, 0);
    cv::flip(colored_frame, colored_frame, 1);

    this->frame = std::move(colored_frame);

    return *this;
  }

  sensor_msgs::ImagePtr
  Result()
  {
    return
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->frame).toImageMsg();
  }

private:
  const std::string& data;
  const uint32_t cols;
  const uint32_t rows;

  cv::Mat frame;
};

sensor_msgs::PointCloud2
ConvertToPointCloud2(
  const zpb::tracker::message::TrackerState& cloud_data
) {
  pcl::PointCloud<ZendarPoint> pointcloud;
  sensor_msgs::PointCloud2 cloud_msg;

  // Fill the PointCloud2
  for (auto point_data : cloud_data.detection()) {
    ZendarPoint point;
    point.x = std::sin(point.az) * std::cos(point.el) * point.r;
    point.y = std::sin(point.el) * point.r;
    point.z = std::cos(point.az) * std::cos(point.el) * point.r;
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

  pcl::toROSMsg(pointcloud, cloud_msg);

  return cloud_msg;
}

geometry_msgs::PoseStamped
ConvertToPoseStamped(
    const zpb::tracker::message::TrackerState& cloud_data
  ) {
  const auto& attitude_proto = cloud_data.meta().attitude();
  const auto& position_proto = cloud_data.meta().position();

  geometry_msgs::PoseStamped pose_stamped_msg;

  pose_stamped_msg.pose.position.x = position_proto.x();
  pose_stamped_msg.pose.position.y = position_proto.y();
  pose_stamped_msg.pose.position.z = position_proto.z();

  pose_stamped_msg.pose.orientation.w = attitude_proto.w();
  pose_stamped_msg.pose.orientation.x = attitude_proto.x();
  pose_stamped_msg.pose.orientation.y = attitude_proto.y();
  pose_stamped_msg.pose.orientation.z = attitude_proto.z();

  return pose_stamped_msg;
}

nav_msgs::OccupancyGrid
ConvertToRosGrid(
  const zpb::drivable_area::OccGridMessage& occ_grid) {

  nav_msgs::OccupancyGrid grid_msg;
  double timestamp = occ_grid.timestamp();
  int width = occ_grid.ncols();
  double res = occ_grid.grid_res();

  grid_msg.header.frame_id = "map";
  grid_msg.header.stamp = ros::Time(timestamp);
  grid_msg.header.seq = occ_grid.seq();
  grid_msg.info.map_load_time = ros::Time(timestamp);
  grid_msg.info.resolution = res;
  grid_msg.info.width = width;
  grid_msg.info.height = width;

  geometry_msgs::Pose pose;
  pose.position.x = -width * res / 2;
  pose.position.y = -width * res / 2;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  grid_msg.info.origin = pose;

  for (int i = 0; i < width * width; i++) {
    int val = occ_grid.grid(i) ?
                  OCCUPIED_VALUE : UNOCCUPIED_VALUE;
    grid_msg.data.push_back(val);
  }
  return grid_msg;
}

}  // namespace



/// \namespace -----------------------------------------------------------------
namespace zen {
ZendarDriverNode::ZendarDriverNode(
  const std::shared_ptr<ros::NodeHandle> node,
  const std::string& url,
  const float max_range,
  int argc,
  char* argv[]
)
  : node(CHECK_NOTNULL(node))
  , url(url)
  , max_range(max_range)
{
  api::ZenApi::Init(&argc, &argv);

  auto default_telem_ports = api::ZenApi::TelemPortOptions();
  api::ZenApi::Connect(url, default_telem_ports);

  auto default_data_ports = api::ZenApi::DataPortOptions();
  api::ZenApi::Bind(default_data_ports);

  api::ZenApi::SubscribeImages();
  api::ZenApi::SubscribeTrackerStates();
  api::ZenApi::SubscribeTracklogs();
  api::ZenApi::SubscribeOccupancyGrid();

  api::ZenApi::SubscribeLogMessages(LOG_MSG_QUEUE);
  api::ZenApi::SubscribeHousekeepingReports();
}

ZendarDriverNode::~ZendarDriverNode()
{
  api::ZenApi::UnsubscribeImages();
  api::ZenApi::UnsubscribeTrackerStates();
  api::ZenApi::UnsubscribeTracklogs();
  api::ZenApi::UnsubscribeOccupancyGrid();

  api::ZenApi::UnsubscribeLogMessages();
  api::ZenApi::UnsubscribeHousekeepingReports();

  api::ZenApi::Release();
  api::ZenApi::Disconnect();
}

void ZendarDriverNode::Run()
{
  ros::Rate loop_rate(LOOP_RATE_HZ);

  // Publish range markers, and ego vehicle once since they are latched topics
  this->ProcessRangeMarkers();
  this->ProcessEgoVehicle();

  // Publish vehicle to map transform
  PublishVehicleToMap();

  while (node->ok()) {
    this->ProcessImages();
    this->ProcessPointClouds();
    this->ProcessOccupancyGrid();
    this->ProcessTracks();
    this->ProcessPoseMessages();
    this->ProcessLogMessages();
    this->ProcessHousekeepingReports();
    loop_rate.sleep();
  }
}

void ZendarDriverNode::ProcessImages()
{
  while (auto image = ZenApi::NextImage(ZenApi::NO_WAIT)) {
    auto image_type = image->cartesian().data().type();
    if (image_type != zpb::data::ImageDataCartesian_Type_REAL_32U) {
      ROS_WARN(
        "Only \"REAL_32U\" image type is supported, got type \"%s\" instead.",
        std::to_string(image_type).c_str());
      continue;
    }

    auto image_normal = ImageNormal(
      *image,
      IMAGE_DOWNSAMPLING_FACTOR,
      IM_DYN_RANGE_MIN,
      IM_DYN_RANGE_MAX
    );

    auto ros_image =
      ImageProcessor(*image)
      .Scale(image_normal, ATAN_SCALE_FACTOR)
      .Color()
      .Result();

    ros_image->header.seq = (uint32_t)(image->meta().frame_id());
    ros_image->header.frame_id = image->meta().serial();
    ros_image->header.stamp = ros::Time(image->meta().timestamp());

    const auto& serial = image->meta().serial();
    this->image_pub.Publish(serial, ros_image);
  }
}

void
ZendarDriverNode::ProcessPointClouds()
{
  while (auto points = ZenApi::NextTrackerState(ZenApi::NO_WAIT)) {
    const auto& serial = points->meta().serial();
    auto cloud2 = ConvertToPointCloud2(*points);

    cloud2.header.seq = (uint32_t)(points->meta().frame_id());
    cloud2.header.frame_id = points->meta().serial();
    cloud2.header.stamp = ros::Time(points->meta().timestamp());

    this->points_pub.Publish(serial, cloud2);

    auto pose_stamped = ConvertToPoseStamped(*points);

    pose_stamped.header.seq = (uint32_t)(points->meta().frame_id());
    pose_stamped.header.stamp = ros::Time(points->meta().timestamp());
    pose_stamped.header.frame_id = "ECEF";

    this->points_metadata_pub.Publish(serial, pose_stamped);
  }
}

void
ZendarDriverNode::ProcessOccupancyGrid()
{
  while(auto occ_grid = ZenApi::NextOccupancyGrid(ZenApi::NO_WAIT)) {
    this->occupancy_grid_pub.publish(ConvertToRosGrid(*occ_grid));
  }
}

void ZendarDriverNode::ProcessTracks()
{
  while (auto tracks = ZenApi::NextTracks(ZenApi::NO_WAIT)) {
    auto tracks_msg = Tracks(*tracks);
    this->tracks_pub.publish(tracks_msg);
  }
}

void ZendarDriverNode::ProcessRangeMarkers()
{
  auto range_markers = RangeMarkers(max_range);
  this->range_markers_pub.publish(range_markers);
}

void ZendarDriverNode::ProcessEgoVehicle()
{
  auto ego_vehicle = EgoVehicle();
  this->ego_vehicle_pub.publish(ego_vehicle);
}

void
ZendarDriverNode::ProcessPoseMessages()
{
  while (auto tracklog = ZenApi::NextTracklog(ZenApi::NO_WAIT)) {

    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.pose.position.x = tracklog->position().x();
    pose_stamped.pose.position.y = tracklog->position().y();
    pose_stamped.pose.position.z = tracklog->position().z();

    pose_stamped.pose.orientation.w = tracklog->attitude().w();
    pose_stamped.pose.orientation.x = tracklog->attitude().x();
    pose_stamped.pose.orientation.y = tracklog->attitude().y();
    pose_stamped.pose.orientation.z = tracklog->attitude().z();

    pose_stamped.header.stamp = ros::Time(tracklog->timestamp());
    pose_stamped.header.frame_id = "ECEF";

    this->pose_pub.publish(pose_stamped);
  }
}

void
ZendarDriverNode::ProcessLogMessages()
{
  while (auto text_log = ZenApi::NextLogMessage(ZenApi::NO_WAIT)) {
    rosgraph_msgs::Log ros_message;
    switch (text_log->severity()) {
      case google::INFO:    ros_message.level = 2;  break;
      case google::WARNING: ros_message.level = 4;  break;
      case google::ERROR:   ros_message.level = 8;  break;
      case google::FATAL:   ros_message.level = 16; break;
      default:              ros_message.level = 16; break;
    }

    ros_message.name = text_log->base_filename();
    ros_message.msg = text_log->message();
    ros_message.file = text_log->full_filename();
    ros_message.line = text_log->file_line();

    ros_message.header.stamp = ros::Time(text_log->timestamp());
    this->logs_pub.publish(ros_message);
  }
}

void
ZendarDriverNode::ProcessHousekeepingReports()
{
  while (auto report = ZenApi::NextHousekeepingReport(ZenApi::NO_WAIT)) {
    this->ProcessHKGpsStatus(*report);
    this->ProcessHKSensorIdentity(*report);
    // other HK subtypes
  }
}

void
ZendarDriverNode::ProcessHKGpsStatus(const zpb::telem::HousekeepingReport& report)
{
  if (report.report_case() != zpb::telem::HousekeepingReport::kGpsStatus) {
    return;
  }
  const auto& message = report.gps_status();

  diagnostic_msgs::DiagnosticArray diagnostics;
  // diagnostics_array.header.stamp = ros::Time(report->timestamp());

  diagnostic_msgs::DiagnosticStatus gps_status;
  gps_status.name = "GPS Status";
  gps_status.level = diagnostic_msgs::DiagnosticStatus::OK;

  diagnostic_msgs::KeyValue num_sats;
  num_sats.key = "Number of satellites";
  num_sats.value = std::to_string(message.qos().satellite_count());
  gps_status.values.push_back(num_sats);

  switch (message.qos().gps_status()) {
    case zpb::GpsFix::NO_FIX:    gps_status.message = "No Fix";             break;
    case zpb::GpsFix::TIME_ONLY: gps_status.message = "Time Only";          break;
    case zpb::GpsFix::FIX_2D:    gps_status.message = "Fix 2D";             break;
    case zpb::GpsFix::FIX_3D:    gps_status.message = "Fix 3D";             break;
    case zpb::GpsFix::SBAS:      gps_status.message = "SBAS";               break;
    default:                     gps_status.message = "Unknown GPS Status"; break;
  }
  diagnostics.status.push_back(gps_status);

  diagnostic_msgs::DiagnosticStatus ins_status;
  ins_status.name = "INS Status";
  ins_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  switch (message.qos().ins_status()) {
    case zpb::data::InsMode::NOT_TRACKING: ins_status.message = "Not Tracking";       break;
    case zpb::data::InsMode::ALIGNING:     ins_status.message = "Aligning";           break;
    case zpb::data::InsMode::TRACKING:     ins_status.message = "Tracking";           break;
    default:                               ins_status.message = "Unknown INS Status"; break;
  }
  diagnostics.status.push_back(ins_status);

  this->pose_quality_pub.publish(diagnostics);
}

void ZendarDriverNode::ProcessHKSensorIdentity(const zpb::telem::HousekeepingReport& report)
{
  if (report.report_case() != zpb::telem::HousekeepingReport::kSensorIdentity) {
    return;
  }

  std::string serial = report.sensor_identity().serial();
  if (serials.find(serial) != serials.end()) {
    // Extrinsic already published
    return;
  }

  PublishExtrinsic(report.sensor_identity());
  serials.insert(serial);
}

void ZendarDriverNode::PublishExtrinsic(const zpb::telem::SensorIdentity& id) {
  geometry_msgs::TransformStamped extrinsic_stamped;
  extrinsic_stamped.header.stamp = ros::Time::now();
  extrinsic_stamped.header.frame_id = "vehicle";
  extrinsic_stamped.child_frame_id = id.serial();
  extrinsic_stamped.transform.translation.x = id.extrinsic().t().x();
  extrinsic_stamped.transform.translation.y = id.extrinsic().t().y();
  extrinsic_stamped.transform.translation.z = id.extrinsic().t().z();
  extrinsic_stamped.transform.rotation.x = id.extrinsic().r().x();
  extrinsic_stamped.transform.rotation.y = id.extrinsic().r().y();
  extrinsic_stamped.transform.rotation.z = id.extrinsic().r().z();
  extrinsic_stamped.transform.rotation.w = id.extrinsic().r().w();

  this->extrinsics_pub.sendTransform(extrinsic_stamped);
}

void ZendarDriverNode::PublishVehicleToMap()
{
  geometry_msgs::TransformStamped veh_to_map_stamped;
  veh_to_map_stamped.header.stamp = ros::Time::now();
  veh_to_map_stamped.header.frame_id = "map";
  veh_to_map_stamped.child_frame_id = "vehicle";
  veh_to_map_stamped.transform.translation.x = 0;
  veh_to_map_stamped.transform.translation.y = 0;
  veh_to_map_stamped.transform.translation.z = 0;
  veh_to_map_stamped.transform.rotation.x = 0;
  veh_to_map_stamped.transform.rotation.y = 0;
  veh_to_map_stamped.transform.rotation.z = 0;
  veh_to_map_stamped.transform.rotation.w = 1;

  this->vehicle_to_map_pub.sendTransform(veh_to_map_stamped);
}

}  // namespace zen
