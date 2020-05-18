#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"

const double PI = 3.1415926535897;



#include <sstream>


geometry_msgs::Point goal;

/** Receives the message of the navigation goal from rviz. */
void receiveNavGoal(const geometry_msgs::Point& new_goal)
{
  goal = new_goal;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "kob_move");
  ros::NodeHandle n;

  ros::Publisher mobile_base_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
  ros::Subscriber goal_sub = n.subscribe("simple/goal", 1, receiveNavGoal); // Máximo 5 mensajes en la cola.
  ros::Rate loop_rate(4);

  geometry_msgs::Twist speed;
  
  
  
  while (ros::ok())
  {
    ROS_INFO("My new goal is: %lf, %lf",goal.x,goal.y);
 

    ros::spinOnce();

    loop_rate.sleep();
    break;
  }


  return 0;
}
