#include "Markers.hpp"
#include <string>


int markers = 0;

visualization_msgs::Marker createLine(double x_origin, double y_origin, double distace_to_colision, double sensor_angle){
  visualization_msgs::Marker line_marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  line_marker.header.frame_id = "/base_link";
  line_marker.header.stamp = ros::Time::now();

  line_marker.ns = "line_marker_" + std::to_string(markers);
  line_marker.id = markers++;

  line_marker.type = visualization_msgs::Marker::ARROW;
  line_marker.action = visualization_msgs::Marker::ADD;

  line_marker.pose.position.x = x_origin;
  line_marker.pose.position.y = y_origin;
  line_marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  line_marker.scale.x = 0.02f;
  line_marker.scale.y = 0.04f;
  line_marker.scale.z = 0.05f;

  // Set the color -- be sure to set alpha to something non-zero!
  line_marker.color.r = 0.0f;
  line_marker.color.g = 0.0f;
  line_marker.color.b = 1.0f;
  line_marker.color.a = 1.5f;

  // marker line points
  geometry_msgs::Point p;
  // first point
  p.x = 0;
  p.y = 0;
  p.z = 0.0;

  line_marker.points.push_back(p);

  p.x += distace_to_colision*cos(sensor_angle);
  p.y += distace_to_colision*sin(sensor_angle);
  line_marker.points.push_back(p);

  line_marker.lifetime = ros::Duration();
  return line_marker;
}


visualization_msgs::Marker createTable(double x, double y, double width, double height){
  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/odom";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "table_marker_"+std::to_string(markers);
  marker.id = markers++;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x*RESOLUTION;
  marker.pose.position.y = y*RESOLUTION;
  marker.pose.position.z = 0.6;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = width*RESOLUTION;
  marker.scale.y = height*RESOLUTION;
  marker.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;

  marker.lifetime = ros::Duration();

  return marker;
}