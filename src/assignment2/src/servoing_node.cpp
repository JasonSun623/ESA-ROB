#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include "pid.h"
#include <chrono>

ros::Publisher pubVelCmd;
geometry_msgs::Pose currentPose;

double rad2deg(double rad) {
    return (rad*(180/M_PI));
}

double deg2rad(double deg) {
    return (deg*M_PI/180);
}

/*
 * Taken from the previous assignment, which took it from this source:
 * https://github.com/aniskoubaa/lab_exams/blob/master/src/shape_drawing/shape_drawing.cpp
 */
void rotate(double dest_angle){
	PID pid = PID(1, 2*M_PI, 2*-M_PI, 1, 0.00, 0.0);

	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);

	double error = pid.calculate(dest_angle, tf::getYaw(currentPose.orientation));
	ROS_INFO("error: %lf", error);
	
	ROS_INFO(" yaw: %lf", tf::getYaw(currentPose.orientation));
	ROS_INFO("dest: %lf", dest_angle);

	while (fabs(error) > deg2rad(0.5)) {
		vel_msg.angular.z = error;
		pubVelCmd.publish(vel_msg);
		error = pid.calculate(dest_angle, tf::getYaw(currentPose.orientation));
		//ROS_INFO("loop: error: %lf", error);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("houdoe");
	
	//force the robot to stop when it reaches the desired angle
	vel_msg.angular.z = 0;
	pubVelCmd.publish(vel_msg);
}

/*
 * Simple move() derived from a function in the previous functions source.
 */
void shoot(double speed, double distance) {
	PID pid = PID(1, 100, -100, 1, 0.00, 0.0);
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(100);

	double cur_pos = 0.0;
	
	double error = distance;

	double t0 = ros::Time::now().toSec();
	
	while(fabs(error) > 0.01) {
		error = pid.calculate(distance, cur_pos);
		ROS_INFO("loop: error: %lf", error);
		double t1 = ros::Time::now().toSec();
		cur_pos = error * (t1-t0);
		vel_msg.linear.x = error;
		pubVelCmd.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	vel_msg.linear.x = 0;
	pubVelCmd.publish(vel_msg);
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

void cbGoal(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    double angle = getAngleBetweenPoses2D(currentPose, msg->pose);
	double distance = getDistBetweenPoses2D(currentPose, msg->pose);
    
    double yaw = tf::getYaw(currentPose.orientation);

    ROS_INFO("planned heading: %lf\n", rad2deg(angle));
    ROS_INFO("pre heading    : %lf\n", rad2deg(yaw));

	rotate(angle);
	
	// always drive forward
	shoot(1.0, distance);
	
	ros::spinOnce();

	double gYaw = tf::getYaw(msg->pose.orientation);
	ROS_INFO("final heading  : %lf\n", rad2deg(gYaw));
	
	ros::spinOnce();
	rotate(gYaw);
}

void cbOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    currentPose = msg->pose.pose;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "servoing_node");
	ros::NodeHandle n;
    ros::Subscriber subGoal = n.subscribe("/goal", 100, cbGoal);
    ros::Subscriber subOdom = n.subscribe("/odom", 100, cbOdom);
    pubVelCmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    
	ros::spin();

	return 0;
}
