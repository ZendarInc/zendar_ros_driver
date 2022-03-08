#include "zendar_ros_driver/zendar_driver_node.h"
#include "zendar_ros_driver/zendar_point.h"

// #include <ros/ros.h>
// #include <diagnostic_msgs/DiagnosticArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <image_transport/image_transport.h>
// #include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>

#include <glog/logging.h>


#include <vector>

using namespace zen;
using namespace zen::api;


/// \namespace -----------------------------------------------------------------
namespace {

constexpr int LOOP_RATE_HZ                 = 100;

constexpr size_t LOG_MSG_QUEUE             = 200;

constexpr size_t IMAGE_DOWNSAMPLING_FACTOR = 5;
constexpr float IM_DYN_RANGE_MAX           = 1000.0; ///< 60dB dynamic range
constexpr float IM_DYN_RANGE_MIN           = 562.0;  ///< 55dB dynamic range

///< max value of image set to 99% of atan's output
const float ATAN_SCALE_FACTOR          = std::tan(0.99 * M_PI_2);

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

    std::size_t downsampled_size = source_data.size() / downsample_factor;
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
    using PixelMap = const uint32_t (*)[this->rows][this->cols];
    const auto pixel_map = *reinterpret_cast<PixelMap>(this->data.data());

    cv::Mat scaled_frame;
    for (int col = 0; col < this->rows; ++col) {
      for (int row = 0; row < this->cols; ++row) {
        float pixel = pixel_map[row][col];
        float scaled_pixel = pixel / normal.med;

        if (scaled_pixel < 1.0) {
          scaled_pixel = 0.0;
        }
        else {
          // subtracting 1 to center 0dB SNR to 0
          scaled_pixel =
            std::atan((scaled_pixel - 1.0) * scale_factor / (normal.limit - 1.0));
        }

        this->frame.at<uint8_t>(row, col) = scaled_pixel * 255 / M_PI_2;
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
  cloud_msg.header.seq = (uint32_t)(cloud_data.meta().frame_id());
  cloud_msg.header.frame_id = cloud_data.meta().serial();
  cloud_msg.header.stamp = ros::Time(cloud_data.meta().timestamp());

  return cloud_msg;
}

geometry_msgs::PoseStamped
ConvertToPoseStamped(
    const zpb::tracker::message::TrackerState& cloud_data
  ) {
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

}  // namespace



/// \namespace -----------------------------------------------------------------
namespace zen {
ZendarDriverNode::ZendarDriverNode(
  const std::shared_ptr<ros::NodeHandle> node,
  const std::string& url,
  int argc,
  char* argv[]
)
  : node(CHECK_NOTNULL(node))
  , url(url)
{
  api::ZenApi::Init(&argc, &argv);

  auto default_telem_ports = api::ZenApi::TelemPortOptions();
  api::ZenApi::Connect(url, default_telem_ports);

  auto default_data_ports = api::ZenApi::DataPortOptions();
  api::ZenApi::Bind(default_data_ports);

  api::ZenApi::SubscribeImages();
  api::ZenApi::SubscribeTrackerStates();
  api::ZenApi::SubscribeTracklogs();

  api::ZenApi::SubscribeLogMessages(LOG_MSG_QUEUE);
  api::ZenApi::SubscribeHousekeepingReports();
}

ZendarDriverNode::~ZendarDriverNode()
{
  api::ZenApi::UnsubscribeImages();
  api::ZenApi::UnsubscribeTrackerStates();
  api::ZenApi::UnsubscribeTracklogs();

  api::ZenApi::UnsubscribeLogMessages();
  api::ZenApi::UnsubscribeHousekeepingReports();

  api::ZenApi::Release();
  api::ZenApi::Disconnect();
}

void ZendarDriverNode::Run()
{
  ros::Rate loop_rate(LOOP_RATE_HZ);

  while (node->ok()) {
    this->ProcessImages();
    this->ProcessPointClouds();
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
      // ROS_WARN(
      //   "Only \"REAL_32U\" image type is supported. "
      //   << "Image type { " + std::to_string(image_type) + " } is not supported."
      // );
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

    ros_image->header.frame_id = "vehicle";
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
    this->points_pub.Publish(serial, cloud2);

    auto pose_stamped = ConvertToPoseStamped(*points);
    this->points_metadata_pub.Publish(serial, pose_stamped);
  }
}

void
ZendarDriverNode::ProcessPoseMessages()
{
  while (auto tracklog = ZenApi::NextTracklog(ZenApi::NO_WAIT)) {

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(tracklog->timestamp());
    pose_stamped.header.frame_id = "ECEF";

    pose_stamped.pose.position.x = tracklog->position().x();
    pose_stamped.pose.position.y = tracklog->position().y();
    pose_stamped.pose.position.z = tracklog->position().z();

    pose_stamped.pose.orientation.w = tracklog->attitude().w();
    pose_stamped.pose.orientation.x = tracklog->attitude().x();
    pose_stamped.pose.orientation.y = tracklog->attitude().y();
    pose_stamped.pose.orientation.z = tracklog->attitude().z();

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

}  // namespace zen
