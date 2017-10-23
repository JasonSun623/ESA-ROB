#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <vector>

ros::Publisher path_publisher;
std::vector<geometry_msgs::PoseStamped> path;

/** converts an angle from degrees to radians **/
double degrees2radians(double angle_in_degrees) {
	return angle_in_degrees * M_PI /180.0;
}

// Jammer dit ROS...
geometry_msgs::PoseStamped makePoseStamped(double pX, double pY, double pZ, double qX, double qY, double qZ, double qW) {
	geometry_msgs::Point p;
	geometry_msgs::Quaternion q;
	geometry_msgs::Pose pos;
	geometry_msgs::PoseStamped posStamped;
	p.x = pX;
	p.y = pY;
	p.z = pZ;
	q.x = qX;
	q.y = qY;
	q.z = qZ;
	q.w = qW;
	pos.position = p;
	pos.orientation = q;
	posStamped.pose = pos;
	return posStamped;
}

void makePath() {
	geometry_msgs::PoseStamped pose = makePoseStamped(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	path.push_back(pose);

	nav_msgs::Path path_msg;
	path_msg.poses = path;

	path_publisher.publish(path_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rectangle_planner_node");
	ros::NodeHandle n;
	path_publisher = n.advertise<nav_msgs::Path>("plan", 1000);
	ros::Rate loop_rate(100);

	makePath();
	
	ros::spin();

	return 0;
}
