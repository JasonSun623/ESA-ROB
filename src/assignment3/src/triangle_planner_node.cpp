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
double rad2deg(double rad) {
    return (rad*(180/M_PI));
}

double deg2rad(double deg) {
    return (deg*M_PI/180);
}

// Jammer dit ROS...
geometry_msgs::PoseStamped makePoseStamped(double pX, double pY, double pZ, double qX, double qY, double qZ, double qW) {
	geometry_msgs::Point p;
	geometry_msgs::Quaternion q;
	geometry_msgs::Pose pos;
	geometry_msgs::PoseStamped ps;

	p.x = pX;
	p.y = pY;
	p.z = pZ;

	q.x = qX;
	q.y = qY;
	q.z = qZ;
	q.w = qW;

	pos.position = p;
	pos.orientation = q;

	ps.pose = pos;

	return ps;
}

void makePath() {
	geometry_msgs::PoseStamped pose = makePoseStamped(6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	path.push_back(pose);
	pose = makePoseStamped(3.0, 5.196152423, 0.0, 0.0, 0.0, 0.8660254, 0.5);
	path.push_back(pose);
	pose = makePoseStamped(0.0, 0.0, 0.0, 0.0, 0.0, 0.8660254, -0.5);
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

	ros::Duration(10.50).sleep();

	makePath();
	
	ros::spin();

	return 0;
}
