#pragma once

#include "zendar_ros_driver/publish.h"


#include <zendar/api/api.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Log.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>


/// \namespace -----------------------------------------------------------------
namespace zen {

class ZendarDriverNode
{
public:
  ZendarDriverNode(
    std::shared_ptr<ros::NodeHandle> node,
    const std::string& url,
    const float max_range,
    int argc,
    char* argv[]
  );

  ~ZendarDriverNode();

  void
  Run();

private:
  void ProcessImages();
  void ProcessPointClouds();
  void ProcessPoseMessages();
  void ProcessLogMessages();
  void ProcessHousekeepingReports();
  void ProcessRangeMarkers();
  void ProcessEgoVehicle();

  void ProcessHKGpsStatus(const zpb::telem::HousekeepingReport& report);

private:
  std::shared_ptr<ros::NodeHandle> node;
  image_transport::ImageTransport image_transport{*this->node};

  RadarImagePublish<sensor_msgs::ImageConstPtr> image_pub{"image_", this->image_transport};
  RadarPublish<sensor_msgs::PointCloud2> points_pub{"points_", *this->node};
  RadarPublish<geometry_msgs::PoseStamped> points_metadata_pub{"points_metadata_", *this->node};

  ros::Publisher pose_pub = this->node->advertise<geometry_msgs::PoseStamped>("/pose", 100);
  ros::Publisher logs_pub = this->node->advertise<rosgraph_msgs::Log>("/zpu_logs", 100);

  ros::Publisher pose_quality_pub = this->node->advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100);

  // Create range marker, and ego vehicle publisher as latched topics
  ros::Publisher range_markers_pub 
    = this->node->advertise<nav_msgs::OccupancyGrid>("/range_markers", 1, true);
  ros::Publisher ego_vehicle_pub = 
    this->node->advertise<visualization_msgs::Marker>("/ego_vehicle", 1, true);

  const std::string url;
  const float max_range;
};
}  ///< \namespace zen
