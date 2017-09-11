#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <assignment1/Triangle.h>
#include <sstream>
#include <math.h>

double distanceTraveled;

void move(double speed, double distance, bool isForward){
	ros::NodeHandle n;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	distanceTraveled += distance;
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		//loop_rate.sleep(); //TODO: Doe terug
	} while(current_distance<distance);
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
}

void rotate (double angular_speed, double relative_angle, bool clockwise){
	ros::NodeHandle n;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(1000);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		//loop_rate.sleep(); //TODO: Doe terug
		//cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	}while(current_angle<relative_angle);

	//force the robot to stop when it reaches the desired angle
	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);
}

/** converts an angle from degrees to radians **/
double degrees2radians(double angle_in_degrees){
	return angle_in_degrees * M_PI /180.0;
}

void timerCallback() {
	
}

void cmdCallback(const assignment1::Triangle::ConstPtr& msg) {
	ROS_INFO("I heard: [%f]", msg->sideLength);
	ROS_INFO("I heard: [%d]", msg->cw);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "draw_triangle_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd", 1000, cmdCallback);

	for (int i = 0; i < 3; i++) {
		distanceTraveled = 0.0;		
		move(1.0, 3.0, true);
		if (i < 2) rotate(1.0, degrees2radians(120.0), false);
	}

	//ros::spin();

	return 0;
}
