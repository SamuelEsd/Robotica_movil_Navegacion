// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <math.h>
#include "World.cpp"
#include "Markers.cpp"
#include "Vector2d.cpp"
// %EndTag(INCLUDES)%


double yaw_angle;
double goal_yaw_angle;

geometry_msgs::Pose kobuki_pose;
geometry_msgs::Pose goal_pose;

bool new_goal = false;
int goal_num = 0;

// -----------------------FUCNIONES AUXILIARES ------------------------------------------------


double getAngleDif(double max, double min){
  double angle_dif = 0;
  if(max < min){
    angle_dif = (max+(2*M_PI))-min;
  }
  else{
    angle_dif = max-min;
  }
  return angle_dif;
}

Vector2d get_resultant_vector(char* data, double x, double y){
  int sensors = 6;
  double sensor_angle = 0;//M_PI*(1/4);
  Vector2d resultant_vector(0,0); 
  if(!filledRectangle(data, y, x)){
    for (int j = 0; j < sensors; j++){
      sensor_angle = ((2*M_PI)/sensors)*j;
      double magnitude = rayTracing(data, x, y, sensor_angle);
      if (magnitude < 5){
        double new_magnitude = 1/magnitude;
        if( new_magnitude > 1.5){
          new_magnitude = 1.5;
        }
        Vector2d temp(-new_magnitude,sensor_angle);
        resultant_vector.sum(temp);
      }
    }
  } 
  return resultant_vector;
}



/** Receives the message of the navigation goal from rviz. */
void receiveNavGoal(const geometry_msgs::PoseStamped& poseStamped)
{
  goal_pose = poseStamped.pose;
  
  tf::Pose pose;
  tf::poseMsgToTF(poseStamped.pose, pose);
  goal_yaw_angle = tf::getYaw(pose.getRotation());

  new_goal = true;
  goal_num = (goal_num+1)%2;
}


/**
 * Get values of position from odom
 */
void receivePosition(const nav_msgs::Odometry::ConstPtr& msg)
{
  kobuki_pose = msg->pose.pose;
  
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  yaw_angle = tf::getYaw(pose.getRotation());
}



// -----------------------METODO PRINCIPAL------------------------------------------------------
// %Tag(INIT)%
int main( int argc, char** argv )
{

  // ---------------------INICIALIZACION DE NODOS NECESARIOS -----------------------------------
  ros::init(argc, argv, "basic_map");
  ros::NodeHandle n;
  ros::Rate r(20);
  // Para publicar los markadores correspondientes a las mesas
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Para publicar el mapa con colores correspondientes a si el cuadro está ocupado o no
  ros::Publisher occupancy_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_marker", 1);

  ros::Publisher goal_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
  // Para suscribirnos al Objetivo al cual deseamos se mueva la kobuki
  ros::Subscriber obj_sub = n.subscribe("/move_base_simple/goal", 1, receiveNavGoal); // Máximo 5 mensajes en la cola.
  // Para suscribirnos a la posición actual de la kobuki
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, receivePosition);
  // %EndTag(INIT)%




  // --------------------INICIALIZACION DEL MAPA ------------------------------------------------
  ROS_INFO("INICIALIZACION DEL MAPA");
  // %Tag(MAP_INIT)%
  nav_msgs::OccupancyGrid map = createMap();

  int size = map.data.size();
  char* data = new char[size];
  for(int i = 0; i < size; i++) {
    data[i] = map.data[i];
  }

  ROS_INFO("INICIALIZACION potential_field");
  std::vector<Vector2d> potential_field;
  
  for(int i = 0; i < size; i++) {
    int y = i%HEIGHT;
    int x = i/HEIGHT;
    potential_field.push_back(get_resultant_vector(data,x+.5,y+.5));
  }

  // %EndTag(MAP_INIT)%

  //----------------- INICIO CREACION DE MARCADORES-------------------------------------------------
  ROS_INFO("INICIO CREACION DE MARCADORES");

  //-----------------------------MESA 0---------------------------------------
  visualization_msgs::Marker marker0 = createTable(HALF_table_height+1,HALF_table_width+1,(HEIGHT_table+1),(WIDTH_table+1)); 
  
  //-------------------------MESA 1---------------------------------------------------------
  visualization_msgs::Marker marker1 = createTable(HALF_table_height+1,(HEIGHT-1)-(HALF_table_width),HEIGHT_table+1,WIDTH_table+1); 
  
  //-------------------------MESA 2-----------------------------------------------
  visualization_msgs::Marker marker2 = createTable((((WIDTH-2)/4)+1)+HALF_table_width,1+HALF_table_height,WIDTH_table+1,HEIGHT_table+1);

  //-------------------------MESA 3-----------------------------------------------
  //                 inicio de mesa más abajo   inicio mesa más arriba    inicio mesa más arriba
  int mid_width = (( ((WIDTH-2)-(WIDTH_table)) - (1+(WIDTH_table)))/2  ) + (1+(WIDTH_table))+1;
  //                   inicio pared pizarrón     inicio de mesa más izquierda       inicio mesa más izquierda
  int mid_height = ( ( (HEIGHT-2)        -       (1+(HEIGHT_table)) )/2  )     +     (1+(HEIGHT_table))+1;

  visualization_msgs::Marker marker3 = createTable(mid_width,mid_height,HEIGHT_table+1,WIDTH_table+1);
  

  //-------------------------MESA 4-----------------------------------------------
  visualization_msgs::Marker marker4 = createTable((WIDTH-1)-HALF_table_width,1+HALF_table_height,WIDTH_table+1,HEIGHT_table+1);

  //-------------------------MESA 5-----------------------------------------------
  visualization_msgs::Marker marker5 = createTable((WIDTH-1)-HALF_table_height,(HEIGHT-1)-((HALF_table_width*2)-0.5),HEIGHT_table+1,(WIDTH_table*2)+1); 


  //------------------------Line Markers array------------------------------------
  int num_sensores = 6;
  std::vector<visualization_msgs::Marker> sensores;

  ROS_INFO("Line markers");

  for (int i = 0; i < num_sensores; i++){
    // Calculate the needed values to create the line marker
    double sensor_angle = (yaw_angle+(((2*M_PI)/num_sensores)*i));
    // As the kobuki position is in the rviz world and it is 
    // scaled we must bring back the position to values without scale
    double position_x = kobuki_pose.position.x*pow(RESOLUTION,-1);
    double position_y = kobuki_pose.position.y*pow(RESOLUTION,-1);
    double distace_to_colision = rayTracing(data, position_x, position_y, sensor_angle);
    
    // Create the marker for an angle
    visualization_msgs::Marker line_marker = createLine(position_x,position_y,distace_to_colision,sensor_angle);
    
    sensores.push_back(line_marker);
  }
  
  
  //------------------------Potential Field Markers------------------------------------

  ROS_INFO("Potential Field Markers");
  std::vector<visualization_msgs::Marker> potential_field_markers;
  for (int i = 0; i < potential_field.size(); i++){
    int y = i%HEIGHT;
    int x = i/HEIGHT;
    double mag = potential_field[i].getMagnitude(); //sqrt(pow(WIDTH,2)*pow(HEIGHT,2));
    if (mag > 0.5){
      mag = 0.5;
    }
    double ang = potential_field[i].getAngle();
    // We have to scale the values to the scale of the rviz world before we 
    // publish them
    visualization_msgs::Marker line_marker = createLine((x+.5)*RESOLUTION,(y+.5)*RESOLUTION,mag*RESOLUTION,ang);
    
    potential_field_markers.push_back(line_marker);
  }

//-------------TESTER------------------------------
  // ROS_INFO("Test Line markers");
  //std::vector<visualization_msgs::Marker> potential_test_markers;
  // Vector2d test = get_resultant_vector(data,1+.5,25.5);
  // double mag = test.getMagnitude(); //sqrt(pow(WIDTH,2)*pow(HEIGHT,2));
  // double ang = test.getAngle();
  // We have to scale the values to the scale of the rviz world before we 
  // publish them
  // visualization_msgs::Marker line_marker = createLine((1+.5)*RESOLUTION,(25+.5)*RESOLUTION,mag*RESOLUTION,ang);
    

  // potential_test_markers.push_back(line_marker);
  // int sensors = 6;
  // double sensor_angle = 0;
  // double x = 7.5;
  // double y = 17.5;
  // for (int j = 0; j < sensors; j++){
  //   sensor_angle = ((2*M_PI)/sensors)*j;
  //   double magnitude = rayTracing(data, x, y, sensor_angle);
  //   ROS_INFO("iteración: %d angulo: %lf magnitud: %lf",j,sensor_angle,magnitude);
  //   visualization_msgs::Marker line_marker = createLine((x)*RESOLUTION,(y)*RESOLUTION,magnitude*RESOLUTION,sensor_angle);
  //   potential_test_markers.push_back(line_marker);
  // } 
  //-------------TESTER------------------------------

  geometry_msgs::Twist speed;

  while (ros::ok())
    {
      occupancy_pub.publish(map);
      marker_pub.publish(marker0);
      marker_pub.publish(marker1);
      marker_pub.publish(marker2);
      marker_pub.publish(marker3);
      marker_pub.publish(marker4);
      marker_pub.publish(marker5);

      // For each line marker update its values and publish it
      for(int i = 0; i < num_sensores; i++){
        double sensor_angle = (yaw_angle+(((2*M_PI)/num_sensores)*i));
        double position_x = kobuki_pose.position.x;
        double position_y = kobuki_pose.position.y;
        sensores[i].pose.position.x = position_x;
        sensores[i].pose.position.y = position_y;
        double distace_to_colision = RESOLUTION*rayTracing(data, position_x*pow(RESOLUTION,-1), position_y*pow(RESOLUTION,-1), sensor_angle);
        sensores[i].points[1].x = distace_to_colision*cos(sensor_angle);
        sensores[i].points[1].y = distace_to_colision*sin(sensor_angle);
        marker_pub.publish(sensores[i]);
      }
      for (int i = 0; i < potential_field.size(); i++){
        marker_pub.publish(potential_field_markers[i]);
      }

      //-------------------------TESTER--------------------------
      // for (int i = 0; i < potential_test_markers.size(); i++){
      //   marker_pub.publish(potential_test_markers[i]);
      // }
      //-------------------------TESTER--------------------------
      // %Tag(SLEEP_END)%
      if(new_goal){
        double x1 = goal_pose.position.x*pow(RESOLUTION,-1);
        double y1 = goal_pose.position.y*pow(RESOLUTION,-1);
        double x2 = kobuki_pose.position.x*pow(RESOLUTION,-1);
        double y2 = kobuki_pose.position.y*pow(RESOLUTION,-1);

        // If the kobuki has just arrived
        if (euclideanDistance(x1,y1,x2,y2) < 0.1){
          new_goal = false;
          speed.linear.x = 0.0;
          speed.angular.z = 0.0;
        }
        else {
          double inc_x = x1 - x2;
          double inc_y = y1 - y2;
          
          Vector2d atraction(inc_x,inc_y,false);
          atraction.setMagnitude(1);

          double angle_to_goal = atraction.getAngle();
          if(yaw_angle < 0){
            yaw_angle += 2*M_PI;
          }

          if (goal_num == 1){
            if (abs(angle_to_goal - yaw_angle) > 0.1){
              double angle_dif = getAngleDif(yaw_angle,angle_to_goal);
              if (angle_dif <= M_PI)
              {
                speed.linear.x = 0.0;
                speed.angular.z = -0.5;  
              }
              else
              {
                speed.linear.x = 0.0;
                speed.angular.z = 0.5;
              }
              
            }
            else{
              speed.linear.x = 0.6;
              speed.angular.z = 0.0;
            }
          } 
          else{
            
            ROS_INFO("x: %lf y: %lf ",inc_x,inc_y);
            ROS_INFO("Potential angle: %lf magnitude: %lf ",atraction.getAngle(),atraction.getMagnitude());
            int i = (HEIGHT*int(floor(x2)))+int(floor(y2));
            atraction.sum(potential_field[i]);
            ROS_INFO("Atraction angle: %lf Yaw angle: %lf ",potential_field[i].getAngle(),yaw_angle);
            ROS_INFO("angle: %lf Magnitude: %lf ",potential_field[i].getAngle(),potential_field[i].getMagnitude());

            ROS_INFO("Resultant angle: %lf Yaw angle: %lf ",atraction.getAngle(),yaw_angle);
            if (abs(atraction.getAngle() - yaw_angle) > 0.1){
              double angle_dif = getAngleDif(yaw_angle,atraction.getAngle());
              speed.linear.x = 0.0;

              if (abs(atraction.getAngle() - yaw_angle) < 1.5){
                speed.linear.x = atraction.getMagnitude()/(angle_dif*10);
                if (angle_dif < 0.1){
                  speed.linear.x = atraction.getMagnitude();
                }
              }

              if (angle_dif <= M_PI)
              { 
                speed.angular.z = -0.3;  
              }
              else
              {
                speed.angular.z = 0.3;
              }
            }
            else{
              int alpha = 1;
              // if(euclideanDistance(x1,y1,x2,y2) < 4){
              //   alpha = euclideanDistance(x1,y1,x2,y2);
              // }
              speed.linear.x = atraction.getMagnitude()*alpha;
              speed.angular.z = 0.0;
            }
            ROS_INFO("linear: %lf angular: %lf ",speed.linear.x,speed.angular.z);
          }
          
          goal_pub.publish(speed);

        }
      }
      ros::spinOnce();
      r.sleep();
    }
  // %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
