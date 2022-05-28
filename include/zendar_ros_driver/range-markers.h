#ifndef RANGE_MARKERS_H_
#define RANGE_MARKERS_H_
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include "zendar/common/protocol.h"


namespace zen {
nav_msgs::OccupancyGrid RangeMarkers(const zpb::tracker::message::TrackerState& tracker_state, const float max_range);
} // namespace zen
#endif // RANGE_MARKERS_H_