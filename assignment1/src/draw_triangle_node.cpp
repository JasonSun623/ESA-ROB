#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <assignment1/Triangle.h>


#include <sstream>

void cmdCallback(const assignment1::Triangle::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->sideLength);
  ROS_INFO("I heard: [%d]", msg->cw);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "draw_triangle_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd", 1000, cmdCallback);
  ros::Publisher vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  while (ros::ok())
  {
  	geometry_msgs::Twist vel;
    vel.angular.z = 1;
    vel.linear.x = 1;

    vel_pub_.publish(vel);    
    ros::spinOnce();
  }

  //ros::spin();

  return 0;
}
