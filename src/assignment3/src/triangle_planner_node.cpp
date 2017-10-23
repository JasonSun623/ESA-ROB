#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <vector>

ros::Publisher path_publisher;

/** converts an angle from degrees to radians **/
double rad2deg(double rad) {
    return (rad*(180/M_PI));
}

double deg2rad(double deg) {
    return (deg*M_PI/180);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "triangle_planner_node");
	ros::NodeHandle n;
	path_publisher = n.advertise<nav_msgs::Path>("plan", 1000);
	ros::Rate loop_rate(100);
	
	ros::spin();

	return 0;
}
