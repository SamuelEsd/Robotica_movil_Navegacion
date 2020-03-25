// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
// %EndTag(INCLUDES)%



/*
------------------PIZARRÓN------------------
|    |                              |       | 
|    |                              |       | 
|    |        TABLE                 |       | 
|    |        -----------------     | "L"   | 
|    |        |       |       |     | TABLE |
|    |        |       |       |     |       |
|    |        |       |       |     |-------|
|    |        |       |       |     |       |
|    |        |       |       |     |       |
|____|        |       |       |     |       |
|             |       |       |     |       |
|             |       |       |     |       |
|             |       |       |     |       |
|             |       |       |     |       |
|             |       |       |     |       |
|             |       |       |     |       |
|             |       |       |     |       |
|____         |_______|_______|     --------.
|    |                                      .Entrada 
|    |                                      . 
|    |         ______________       ________.
|    |        |              |     |        |
|    |        |              |     |        |
|    |        |              |     |        |
---------------------------------------------
 */



/// El eje X es rojo.
/// El eje Y es verde.
/// El eje Z apunta hacia arriba y el marcador es azul.

const int WIDTH = 24;      /// A lo largo del eje rojo x
const int HEIGHT = 30;     /// A lo largo del eje verde

const int WIDTH_unit = WIDTH/6;      /// A lo largo del eje rojo x
const int HEIGHT_unit = HEIGHT/6;     /// A lo largo del eje verde

/** Sets the cells between [i1,j1] and [i2,j2] inclusive as occupied with probability value. */
void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value)
{
  for(int i = i1; i <= i2; i++)
  {
    for(int j = j1; j <= j2; j++)
    {
      data[i*WIDTH+j] = value;
    }
  }
}

/** Receives the message of the navigation goal from rviz. */
void receiveNavGoal(const geometry_msgs::PoseStamped& poseStamped)
{
  ROS_INFO("\nFrame: %s\nMove to: [%f, %f, %f] - [%f, %f, %f, %f]",
           poseStamped.header.frame_id.c_str(),
           poseStamped.pose.position.x,
           poseStamped.pose.position.y,
           poseStamped.pose.position.z,
           poseStamped.pose.orientation.x,
           poseStamped.pose.orientation.y,
           poseStamped.pose.orientation.z,
           poseStamped.pose.orientation.w);
}


// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_map");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Publisher occupancy_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_marker", 1);
  ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 5, receiveNavGoal); // Máximo 5 mensajes en la cola.
// %EndTag(INIT)%

// %Tag(MAP_INIT)%
  nav_msgs::OccupancyGrid map;

  // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
  map.header.frame_id = "/odom";
  map.header.stamp = ros::Time::now();   // No caduca
  
  map.info.resolution = 0.3;             // [m/cell]
  map.info.width = WIDTH;                // [cells]
  map.info.height = HEIGHT;              // [cells]
  map.info.origin.position.x = 0;
  map.info.origin.position.y = 0;
  map.info.origin.position.z = 0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  
  //int8[] &_data = &map.data
  int size = WIDTH * HEIGHT;
  char* data = new char[size];
  for(int i = 0; i < size; i++) {
    data[i] = 0;
  }

  data[0] = 50;                            // El origen está en la esquina inferior izquierda.
  fillRectangle(data, 0, 1, 0, WIDTH-1, 100);   // Renglón 0. Las columnas van de 0 a WIDTH-1.  Los renglones corren sobre el eje Y.
  fillRectangle(data, 1, 0, HEIGHT-1, 0, 100);  // Columna 0. Los renglones va de 0 a HEIGHT-1.  Las columnas corren sobre el eje X.

  fillRectangle(data, 0, 0, (HEIGHT_unit*2)-1, WIDTH_unit-1, 100);
  fillRectangle(data, (HEIGHT_unit*4)-1, 1, (HEIGHT_unit*6)-1, WIDTH_unit-1, 100);

  fillRectangle(data, 0, (WIDTH_unit*2)-1, (HEIGHT_unit*1)-1, (WIDTH_unit*5)-1, 100);
  fillRectangle(data, (HEIGHT_unit*2)-1, (WIDTH_unit*2)-1, (HEIGHT_unit*5)-1, (WIDTH_unit*4)-1, 100);    

  fillRectangle(data, 0, (WIDTH_unit*5)-1, (HEIGHT_unit*1)-1, (WIDTH_unit*6)-1, 100);
  fillRectangle(data, (HEIGHT_unit*2)-1, (WIDTH_unit*5)-1, (HEIGHT_unit*6)-1, (WIDTH_unit*6)-1, 100);    
    
    
  fillRectangle(data, HEIGHT-1, 1, HEIGHT-1, WIDTH-1, 100);   // Renglón ultimo. Las columnas van de 0 a WIDTH-1.  Los renglones corren sobre el eje Y+height-1.
  fillRectangle(data, 1, WIDTH-1, HEIGHT-1, WIDTH-1, 100);  // Columna ultima. Los renglones va de 0 a HEIGHT-1.  Las columnas corren sobre el eje X+wheight-1.

    
  map.data = std::vector<int8_t>(data, data + size);
  
  // %EndTag(MAP_INIT)%





  // %Tag(MARKER_INIT)%
  visualization_msgs::Marker marker0;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker0.header.frame_id = "/odom";
  marker0.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker0.ns = "table_marker_0";
  marker0.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker0.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker0.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker0.pose.position.x = (WIDTH_unit*0.3)/2;
  marker0.pose.position.y = HEIGHT_unit*0.3;
  marker0.pose.position.z = 0.6;
  marker0.pose.orientation.x = 0.0;
  marker0.pose.orientation.y = 0.0;
  marker0.pose.orientation.z = 0.0;
  marker0.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker0.scale.x = WIDTH_unit*0.3;
  marker0.scale.y = HEIGHT_unit*0.3*2;
  marker0.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker0.color.r = 0.0f;
  marker0.color.g = 1.0f;
  marker0.color.b = 0.0f;
  marker0.color.a = 1.0;

  marker0.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%


  // %Tag(MARKER_INIT)%
  visualization_msgs::Marker marker1;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker1.header.frame_id = "/odom";
  marker1.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker1.ns = "table_marker_1";
  marker1.id = 1;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker1.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker1.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker1.pose.position.x = (WIDTH_unit*0.3)/2;
  marker1.pose.position.y = HEIGHT_unit*0.3*5;
  marker1.pose.position.z = 0.6;
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker1.scale.x = WIDTH_unit*0.3;
  marker1.scale.y = HEIGHT_unit*0.3*2;
  marker1.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker1.color.r = 0.0f;
  marker1.color.g = 1.0f;
  marker1.color.b = 0.0f;
  marker1.color.a = 1.0;

  marker1.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%




    // %Tag(MARKER_INIT)%
  visualization_msgs::Marker marker2;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker2.header.frame_id = "/odom";
  marker2.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker2.ns = "table_marker_2";
  marker2.id = 2;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker2.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker2.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker2.pose.position.x = (WIDTH_unit*0.3)*3;
  marker2.pose.position.y = (HEIGHT_unit*0.3)*3.5;
  marker2.pose.position.z = 0.6;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker2.scale.x = WIDTH_unit*0.3*2;
  marker2.scale.y = HEIGHT_unit*0.3*3;
  marker2.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker2.color.r = 0.0f;
  marker2.color.g = 1.0f;
  marker2.color.b = 0.0f;
  marker2.color.a = 1.0;

  marker2.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%




  // %Tag(MARKER_INIT)%
  visualization_msgs::Marker marker3;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker3.header.frame_id = "/odom";
  marker3.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker3.ns = "table_marker_3";
  marker3.id = 3;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker3.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker3.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker3.pose.position.x = (WIDTH_unit*0.3)*4;
  marker3.pose.position.y = (HEIGHT_unit*0.3)*0.5;
  marker3.pose.position.z = 0.6;
  marker3.pose.orientation.x = 0.0;
  marker3.pose.orientation.y = 0.0;
  marker3.pose.orientation.z = 0.0;
  marker3.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker3.scale.x = WIDTH_unit*0.3*4;
  marker3.scale.y = HEIGHT_unit*0.3;
  marker3.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker3.color.r = 0.0f;
  marker3.color.g = 1.0f;
  marker3.color.b = 0.0f;
  marker3.color.a = 1.0;

  marker3.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%


    // %Tag(MARKER_INIT)%
  visualization_msgs::Marker marker4;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker4.header.frame_id = "/odom";
  marker4.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker4.ns = "table_marker_4";
  marker4.id = 4;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker4.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker4.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker4.pose.position.x = (WIDTH_unit*0.3)*5.5;
  marker4.pose.position.y = (HEIGHT_unit*0.3)*4;
  marker4.pose.position.z = 0.6;
  marker4.pose.orientation.x = 0.0;
  marker4.pose.orientation.y = 0.0;
  marker4.pose.orientation.z = 0.0;
  marker4.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker4.scale.x = WIDTH_unit*0.3;
  marker4.scale.y = HEIGHT_unit*0.3*4;
  marker4.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker4.color.r = 0.0f;
  marker4.color.g = 1.0f;
  marker4.color.b = 0.0f;
  marker4.color.a = 1.0;

  marker4.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%

  
  while (ros::ok())
    {
      occupancy_pub.publish(map);
      marker_pub.publish(marker0);
      marker_pub.publish(marker1);
      marker_pub.publish(marker2);
      marker_pub.publish(marker3);
      marker_pub.publish(marker4);
      // %Tag(SLEEP_END)%
      ros::spinOnce();
      r.sleep();
    }
  // %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
