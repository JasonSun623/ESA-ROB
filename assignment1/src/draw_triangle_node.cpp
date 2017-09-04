#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#include <sstream>

float sideLength;
bool cw;

void cmdCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "draw_triangle_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd", 1000, cmdCallback);

  /*while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    cmd_sub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }s
*/
  ros.spin();

  return 0;
}
