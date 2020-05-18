//Markers.h

#ifndef __MARKERS_HPP_INCLUDED__   
#define __MARKERS_HPP_INCLUDED__   

#include <visualization_msgs/Marker.h>

visualization_msgs::Marker createTable(double x, double y, double width, double height); 
visualization_msgs::Marker createLine(double x_origin, double y_origin, double distace_to_colision, double sensor_angle);

#endif 