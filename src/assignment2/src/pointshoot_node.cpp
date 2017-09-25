#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

ros::Publisher pubVelCmd;
geometry_msgs::Pose currentPose;

// taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
static void toEulerianAngle( geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) {
    // roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
	    pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	yaw = atan2(siny, cosy);
}

/*
 * Taken from the previous assignment, which took it from this source:
 * 
 */
void rotate (double angular_speed, double relative_angle){	
	geometry_msgs::Twist vel_msg;

	vel_msg.angular.z = abs(angular_speed);

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(1000);
	do{
		pubVelCmd.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);
	
	//force the robot to stop when it reaches the desired angle
	vel_msg.angular.z = 0;
	pubVelCmd.publish(vel_msg);
}

double getAngleBetweenPoses2D(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    float dx = p2.position.x - p1.position.x;
    float dy = p2.position.y - p1.position.y;
    return (atan2(dx, dy));
}

double rad2deg(double rad) {
    return (rad*(180/M_PI));
}

void cbGoal(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    geometry_msgs::Pose goal = msg->pose;

    double angle = getAngleBetweenPoses2D(currentPose, goal);

    
    double pitch, roll, yaw;

    toEulerianAngle(currentPose.orientation, pitch, roll, yaw);

    ROS_INFO("planned_heading: %lf\n", rad2deg(angle));
    ROS_INFO("my_heading(yaw): %lf\n", rad2deg(yaw));
    
    rotate(1.0, angle-yaw);

}

void cbOdom(const nav_msgs::Odometry::ConstPtr &msg) {
    currentPose = msg->pose.pose;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pointshoot_node");
	ros::NodeHandle n;
    ros::Subscriber subGoal = n.subscribe("/goal", 100, cbGoal);
    ros::Subscriber subOdom = n.subscribe("/odom", 100, cbOdom);
    pubVelCmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
	ros::spin();

	return 0;
}

