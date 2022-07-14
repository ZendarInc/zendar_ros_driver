#include "zendar_ros_driver/tracks.h"
namespace zen {
visualization_msgs::MarkerArray
Tracks(const zpb::drivable_area::Tracks& tracks){
  visualization_msgs::MarkerArray track_msgs;
  for (const zpb::drivable_area::Track& track : tracks.track()) {
    // Iterate over edges of bounding box (= bounding box visualization
    // represented by list of edges msgs)
    for (std::size_t edge_id = 0; edge_id < track.bbox_edges().size() - 1;
         edge_id += 2)
    {
      edge_msg = CreateEdgeMsg(track, edge_id);
      track_msgs.markers.push_back(edge_msg);
    }
    velocity_msg = CreateVelocityMsg(track);

    track_msgs.markers.push_back(velocity_msg);
  }
  return track_msgs
}

visualization_msgs::Marker
CreateEdgeMsg(const zpb::drivable_area::Track& track, int& edge_id) {
  visualization_msgs::Marker edge_msg;

  // Define header
  edge_msg.header.frame_id = "map";
  edge_msg.header.stamp = ros::Time::now();

  edge_msg.ns = std::to_string(track.id());
  edge_msg.id = edge_id;

  edge_msg.type = visualization_msgs::Marker::ARROW;
  edge_msg.action = visualization_msgs::Marker::ADD;
  edge_msg.lifetime = ros::Duration(0);

  // Define pose (Identity since the actual pose of the edge is specified by
  // its start and end point)
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  edge_msg.pose = pose;

  // Define scale (Only want shaft of arrow => Set head diameter/length to 0)
  edge_msg.scale.x = EDGE_DIAMETER;  // shaft diameter
  edge_msg.scale.y = 0;  // head diameter
  edge_msg.scale.z = 0;  // head length

  // Define color for each line
  std_msgs::ColorRGBA color_msg;
  edge_msg.color.r = track.color().x();
  edge_msg.color.g = track.color().y();
  edge_msg.color.b = track.color().z();
  edge_msg.color.a = 1;

  // Define start and end of arrow
  geometry_msgs::Point start_point_msg;
  start_point_msg.x = track.bbox_edges()[edge_id].x();
  start_point_msg.y = track.bbox_edges()[edge_id].y();
  start_point_msg.z = track.bbox_edges()[edge_id].z();

  // The next steps are needed to counteract the non-existing arrow head
  std::vector<float> edge{0, 0, 0};
  egde[0] = track.bbox_edges()[edge_id+1].x() - track.bbox_edges()[edge_id].x();
  egde[1] = track.bbox_edges()[edge_id+1].y() - track.bbox_edges()[edge_id].y();
  egde[2] = track.bbox_edges()[edge_id+1].z() - track.bbox_edges()[edge_id].z();

  geometry_msgs::Point end_point_msg;
  // The default shaft:head ratio in rviz is 1:0.3 (see rviz repo).
  // Therefore, mulitply edge by the following scale factor to get an arrow
  // only consisting of a body (without a head) the same length as the edge
  float scale_factor = 1 / (1 - 0.3);
  end_point_msg.x = track.bbox_edges()[edge_id].x() + scale_factor * edge[0];
  end_point_msg.y = track.bbox_edges()[edge_id].y() + scale_factor * edge[1];
  end_point_msg.z = track.bbox_edges()[edge_id].z() + scale_factor * edge[2];

  edge_msg.points.push_back(start_point_msg);
  edge_msg.points.push_back(end_point_msg);

  return edge_msg;
}

visualization_msgs::Marker
CreateVelocityMsg(const zpb::drivable_area::Track& track) {
  visualization_msgs::Marker velocity_msg;

  // Define header
  velocity_msg.header.frame_id = "map";
  velocity_msg.header.stamp =ros::Time::now();

  velocity_msg.type = visualization_msgs::Marker::ARROW;
  velocity_msg.action = visualization_msgs::Marker::ADD;
  velocity_msg.lifetime = ros::Duration(0);
  velocity_msg.ns = std::to_string(track.id());
  velocity_msg.id = -1;

  // Define scale
  velocity_msg.scale.x = VELOCITY_SHAFT_DIAMETER;  // shaft diameter
  velocity_msg.scale.y = VELOCITY_HEAD_DIAMETER;  // head diameter
  velocity_msg.scale.z = VELOCITY_HEAD_LENGTH;  // head length

  // Define color
  velocity_msg.color.r = track.color().x();
  velocity_msg.color.g = track.color().y();
  velocity_msg.color.b = track.color().z();
  velocity_msg.color.a = 1;

  // Define start and end point of arrow
  geometry_msgs::Point start_point_msg;
  start_point_msg.x = track.bbox_center().x();
  start_point_msg.y = track.bbox_center().y();
  start_point_msg.z = track.bbox_center().z();

  geometry_msgs::Point end_point_msg;
  end_point_msg.x = track.bbox_center().x() + track.velocity().x();
  end_point_msg.y = track.bbox_center().y() + track.velocity().y();
  end_point_msg.z = track.bbox_center().z() + track.velocity().z();

  velocity_msg.points.push_back(start_point_msg);
  velocity_msg.points.push_back(end_point_msg);

  return velocity_msg;
}
}  // namespace zen