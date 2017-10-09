#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

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
void rotate(double angular_speed, double relative_angle){	

	double ang_speed_sgn = angular_speed > 0.0 ? 1.0 : -1.0;

	geometry_msgs::Twist vel_msg;

	vel_msg.angular.z = angular_speed;

	double t0 = ros::Time::now().toSec();
	double rotated_angle = 0.0;
	ros::Rate loop_rate(100);

	while (rotated_angle < relative_angle) {
		pubVelCmd.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		rotated_angle = ang_speed_sgn * angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	//force the robot to stop when it reaches the desired angle
	vel_msg.angular.z = 0;
	pubVelCmd.publish(vel_msg);
}

/*
 * Simple move() derived from a function in the previous functions source.
 */
void shoot(double speed, double distance) {
	speed = fabs(speed);
	geometry_msgs::Twist vel_msg;
	
	vel_msg.linear.x = speed;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do {
		pubVelCmd.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_distance<distance);
	
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

	double smallest_angle = angles::shortest_angular_distance(angle, yaw);

	double speed = 1.0;
	if (smallest_angle < 0.0) speed = -1.0;
	rotate(speed, fabs(smallest_angle));

	yaw = tf::getYaw(currentPose.orientation);
	ROS_INFO("post_heading   : %lf\n", rad2deg(yaw));
	
	// always drive forward
	shoot(1.0, distance);
	
	// update again
	ros::spinOnce();
	yaw = tf::getYaw(currentPose.orientation);	

	double gYaw = tf::getYaw(msg->pose.orientation);
	ROS_INFO("final heading  : %lf\n", rad2deg(gYaw));
	
	double final_angle = angles::shortest_angular_distance(gYaw, yaw);
	speed = 1.0;
	if (final_angle < 0.0) speed = -1.0;
	rotate(speed, fabs(final_angle));
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
