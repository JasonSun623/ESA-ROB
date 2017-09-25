#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

ros::Publisher pubVelCmd;
geometry_msgs::Pose currentPose;

double getAngleBetweenPoses2D(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    float dx = p2.position.x - p1.position.x;
    float dy = p2.position.y - p1.position.y;
    return (atan2(dx, dy));
}

void cbGoal(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    geometry_msgs::Pose goal = msg->pose;

    double angle = getAngleBetweenPoses2D(currentPose, goal);

	ROS_INFO("angle: %lf\n", angle);
    
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

