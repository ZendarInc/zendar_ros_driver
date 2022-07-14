#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <zendar/api/api.h>

namespace zen {
// Diameter of the bounding box edges
static constexpr float EDGE_DIAMETER = 0.3;
// Diameter of the velocity arrow's shaft
static constexpr float VELOCITY_SHAFT_DIAMETER = 0.1;
// Diameter of the velocity arrow's head
static constexpr float VELOCITY_HEAD_DIAMETER = 0.3;
// Length of the velocity arrow's head
static constexpr float VELOCITY_HEAD_LENGTH = 0.3;

visualization_msgs::MarkerArray Tracks(const zpb::drivable_area::Tracks& tracks);
visualization_msgs::Marker CreateEdgeMsg(const zpb::drivable_area::Track& track,
                                         int& edge_id);
visualization_msgs::Marker CreateVelocityMsg(const zpb::drivable_area::Track& track);
}  // namespace zen