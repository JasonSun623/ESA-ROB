#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <math.h>

ros::Publisher path_publisher;

/** converts an angle from degrees to radians **/
double degrees2radians(double angle_in_degrees) {
	return angle_in_degrees * M_PI /180.0;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rectangle_planner_node");
	ros::NodeHandle n;
	path_publisher = n.advertise<nav_msgs::Path>("plan", 1000);
	ros::Rate loop_rate(100);
	
	ros::spin();
}
