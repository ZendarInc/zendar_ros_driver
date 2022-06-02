#ifndef EGO_VEHICLE_H_
#define EGO_VEHICLE_H_
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <zendar/api/api.h>

namespace zen {
visualization_msgs::Marker EgoVehicle();
} // namespace zen
#endif // EGO_VEHICLE_H_