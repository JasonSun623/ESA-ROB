#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "assignment1/Triangle.h"
#include <sstream>
#include <math.h>

ros::Publisher velocity_publisher;

void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;

	if (isForward)
		vel_msg.linear.x = abs(speed);
	else
		vel_msg.linear.x = -abs(speed);

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_distance<distance);
	
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);

}

void rotate (double angular_speed, double relative_angle, bool clockwise){	
	geometry_msgs::Twist vel_msg;

	if (clockwise)
		vel_msg.angular.z = -abs(angular_speed);
	else
		vel_msg.angular.z = abs(angular_speed);

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(1000);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);
	
	//force the robot to stop when it reaches the desired angle
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

/** converts an angle from degrees to radians **/
double degrees2radians(double angle_in_degrees){
	return angle_in_degrees * M_PI /180.0;
}

void cmdCallback(const assignment1::Triangle::ConstPtr& msg) {
	ROS_INFO("I heard: [%f]", msg->sideLength);
	ROS_INFO("I heard: [%d]", msg->cw);

	bool cw = msg->cw;
	double moveSpeed = 2.0;
	double rotSpeed = 2.0;

	double moveDist = msg->sideLength;
	double rotRadians = degrees2radians(120.0);

	move(moveSpeed, moveDist, true);
	rotate(rotSpeed, rotRadians, cw);
	move(moveSpeed, moveDist, true);
	rotate(rotSpeed, rotRadians, cw);
	move(moveSpeed, moveDist, true);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "draw_triangle_node");
	ros::NodeHandle n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("cmd", 1000, cmdCallback);
	
	printf("pls\n");
	ros::spin();

	return 0;
}
