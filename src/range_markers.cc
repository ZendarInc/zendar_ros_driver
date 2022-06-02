#include "zendar_ros_driver/range_markers.h"

namespace zen {
nav_msgs::OccupancyGrid RangeMarkers(
    const zpb::tracker::message::TrackerState& tracker_state, float max_range){
  int num_circles = floor(max_range / 10);
  // Define the width, and height of the image (in pixel). This way of 
  // computing the number of pixels is done since for a max_range of 40, a 
  // 1001x1001-pixel image provides a decent resolution, and all other values
  // such as the offset of the text to the range marker were choosen based on
  // this resolution.
  int width = max_range / 40.0 * 1001;
  int height = max_range / 40.0 * 1001;

  int additional_boundary = 100;
  int image_width = width + additional_boundary;
  int image_height = height + additional_boundary;
  float resolution = 2 * max_range / (float) width;
  int origin_offset = floor(image_width / 2.0);
  float origin_x = - origin_offset * resolution;
  float origin_y = - origin_offset * resolution;
  int image_origin_x = round(image_width - origin_offset);
  int image_origin_y = image_origin_x; 
  cv::Point image_center(image_origin_y, image_origin_x);
  // Because ros/webviz flips the colors, we draw black circles on a white 
  // image
  cv::Mat img = 255 * cv::Mat::ones(image_height, image_width, CV_8UC1);
  int radius_increase = floor(width / (float) (2 * num_circles));
  // Draw circles, and corresponding distances in image
  for (int i = 1; i < num_circles + 1; i++){
    int circle_radius = round(i * radius_increase);
    cv::circle(img, image_center, circle_radius, cv::Scalar(0), 5);
    cv::Point image_center_shifted = cv::Point(image_origin_y - 50,
                                               image_origin_x - circle_radius - 20);
    float displayed_range = round(i * max_range / num_circles * 100.0) / 100.0;
    // Set display format for range
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << displayed_range << " m";
    cv::putText(img, stream.str(), image_center_shifted,
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0), 3);
  }
  // Flip, and rotate image to counter transformations made in ros/webviz
  cv::flip(img, img, 0);
  cv::rotate(img, img, cv::ROTATE_90_COUNTERCLOCKWISE);

  nav_msgs::OccupancyGrid grid_msg;
  // Define header
  grid_msg.header.frame_id = "map";
  grid_msg.header.stamp = ros::Time(tracker_state.meta().timestamp());
  grid_msg.header.seq = (uint32_t)(tracker_state.meta().frame_id());
  // Define metadata
  grid_msg.info.map_load_time = ros::Time(tracker_state.meta().timestamp());
  grid_msg.info.resolution = resolution;
  grid_msg.info.width = image_width;
  grid_msg.info.height = image_height;
  // Define origin of map
  geometry_msgs::Pose pose;
  pose.position.x = origin_x;
  pose.position.y = origin_y;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  grid_msg.info.origin = pose;
  cv::MatIterator_<uchar> it, end;
  for(it = img.begin<uchar>(), end = img.end<uchar>(); it != end; ++it){
      // Convert pixel to probabilty (needed for nav_msgs::OccupancyGrid)
      int pixel = 100 * it[0] / 255.0;
      grid_msg.data.push_back(pixel);
  }
  return grid_msg;
}
} // namespace zen