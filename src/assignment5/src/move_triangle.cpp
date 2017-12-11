#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "assignment5/Triangle.h"
#include <sstream>
#include <math.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/Pose.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>

ros::Publisher velocity_publisher;
ros::ServiceClient pen_srv;
turtlesim::Pose turtlesim_pose;
turtlesim::SetPen pen;

const double window_min_x = 0.0;
const double window_min_y = 0.0;

const double window_max_x = 11.0;
const double window_max_y = 11.0;



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
double degrees2radians(double angle_in_degrees) {
	return angle_in_degrees * M_PI /180.0;
}


void setInitialOrientation (double desired_angle_radians){
	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	ROS_INFO("rot func: %f\n", turtlesim_pose.theta);
	ROS_INFO("rel rot: %f\n", relative_angle_radians);
	ros::spinOnce();

	bool clockwise = ((relative_angle_radians<0)?true:false);
	rotate (2.0, abs(relative_angle_radians), clockwise);
}


void cmdCallback(const assignment5::Triangle::ConstPtr& msg) {
	ros::Rate loop_rate(100);
	pen.request.off = 0;

	while (!pen_srv.call(pen)) {
		loop_rate.sleep();
	}



	ROS_INFO("I heard: [%f]", msg->sideLength);
	ROS_INFO("I heard: [%d]", msg->cw);

	bool cw = msg->cw;
	double moveSpeed = 2.0;
	double rotSpeed = 2.0;

	double moveDist = msg->sideLength;
	double rotRadians = degrees2radians(120.0);

	float addX = moveDist * cos(rotRadians);
	float addY = moveDist * sin(rotRadians);

	double finalX = turtlesim_pose.x + addX;
	double finalY = turtlesim_pose.y + addY;
	ROS_INFO("FinalX: %f\n", finalX);
	ROS_INFO("FinalY: %f\n", finalY);
	

	if (finalX > window_max_x) {
		moveDist = window_max_x - turtlesim_pose.x;
	}
	if (finalY > window_max_y) {
		moveDist = window_max_y - turtlesim_pose.y;
	}
	if (finalX < window_min_x) {
		moveDist = window_min_x - turtlesim_pose.x;
	}
	if (finalY < window_min_y) {
		moveDist = window_min_y - turtlesim_pose.y;
	}
	

	move(moveSpeed, moveDist, true);
	rotate(rotSpeed, rotRadians, cw);
	move(moveSpeed, moveDist, true);
	rotate(rotSpeed, rotRadians, cw);
	move(moveSpeed, moveDist, true);
	
	//rotate(rotSpeed, rotRadians, cw);

	ROS_INFO("rot pre: %f\n", turtlesim_pose.theta);

	// fixes everything
	while (turtlesim_pose.theta > 0.01 || turtlesim_pose.theta < -0.01) {
		ros::spinOnce();
		setInitialOrientation(0.0);
	}

	ROS_INFO("rot post: %f\n", turtlesim_pose.theta);
	ros::spinOnce();
	pen.request.off = 1;
	while (!pen_srv.call(pen)) {
		loop_rate.sleep();
	}
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message) {
	/********* TODO *******************************
	 * 1- update the turtlesim_pose global variable with
	 * the pose received in the message pose_message
	 * of the the callback function
	 *********************************************/

	turtlesim_pose.x = pose_message->x; //change the value to make it correct
	turtlesim_pose.y = pose_message->y;//change the value to make it correct
	turtlesim_pose.theta = pose_message->theta;//change the value to make it correct
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "draw_triangle_node");
	ros::NodeHandle n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("cmd", 1000, cmdCallback);
	ros::Subscriber poseSubscriber = n.subscribe("/turtle1/pose", 1000, poseCallback);
	ros::Rate loop_rate(100);
	
	pen_srv = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
	pen.request.r = 255;
	pen.request.g = 0;
	pen.request.b = 0;
	pen.request.width = 0;	
	while (!pen_srv.call(pen)) {
		loop_rate.sleep();
	}
	
	ros::spin();

	return 0;
}
