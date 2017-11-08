/*
 * Follow-the-carrot node
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <nav_msgs/Path.h>
#include "pid.h"
#include <turtlesim/Spawn.h>
#include <tf/transform_listener.h>

const double lookAhead = 1.337;
ros::Publisher g_velPublisher;
geometry_msgs::Pose g_currentPose;
std::vector<geometry_msgs::PoseStamped, std::allocator<geometry_msgs::PoseStamped>> g_goals;

double rad2deg(double rad) {
    return (rad*(180/M_PI));
}

double deg2rad(double deg) {
    return (deg*M_PI/180);
}

double getAngleBetweenPoses2D(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    float dx = p2.position.x - p1.position.x;
    float dy = p2.position.y - p1.position.y;
    return (atan2(dy, dx));
}

double getDistBetweenPoses2D(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    float dx = p2.position.x - p1.position.x;
	float dy = p2.position.y - p1.position.y;
    return (sqrt(dx*dx+dy*dy));
}

void cbPath(const nav_msgs::Path::ConstPtr &msg) {
	g_goals = msg->poses;
	ROS_INFO("Path sz: %d", (int)g_goals.size());
	for (auto goal : g_goals) {
		ROS_INFO("x: %lf, y: %lf", goal.pose.position.x, goal.pose.position.y);		
		geometry_msgs::PoseStamped::ConstPtr goalPtr(new geometry_msgs::PoseStamped(goal));
	}
}

void updatePose(const tf::StampedTransform &transform) {
	g_currentPose.position.x = transform.getOrigin().x();
	g_currentPose.position.y = transform.getOrigin().y();
	g_currentPose.position.z = transform.getOrigin().z();
	tf::quaternionTFToMsg(transform.getRotation(), g_currentPose.orientation);

	ROS_INFO("pos: x: %lf, y: %lf, z: %lf", 
		g_currentPose.position.x, 
		g_currentPose.position.y, 
		g_currentPose.position.z);		
	ROS_INFO("rot: x: %lf, y: %lf, z: %lf, w: %lf", 
		g_currentPose.orientation.x, 
		g_currentPose.orientation.y, 
		g_currentPose.orientation.z, 
		g_currentPose.orientation.w);
}

geometry_msgs::Pose findIntersect(double distance) {
	geometry_msgs::Pose goal;
	//y = a * x + b

}

void moveToNextPosition() {
	auto nextIntersection = findIntersect(lookAhead);
	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "follow_carrot_node");
	ros::NodeHandle n;
	
	ros::Subscriber subPath = n.subscribe("/plan", 100, cbPath);
	g_velPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	ros::Rate rate(10.0);
	tf::TransformListener listener;
	
	while (n.ok()) {
		tf::StampedTransform transform;
		try {
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			updatePose(transform);
			moveToNextPosition();
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		//ros::spinOnce();
		rate.sleep();
		
	}

	return 0;
}
