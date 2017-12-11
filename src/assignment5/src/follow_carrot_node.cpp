/*
 * Follow-the-carrot node
 */

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// limit the speed to 1.0 units/second
const double g_maxSpeed = 1.0;
const double g_lookAhead = 0.75;
const double g_tolerance = 0.01;
const double g_gain = 4.0;
const double g_pow = 3.0;
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
    double dx = p2.position.x - p1.position.x;
    double dy = p2.position.y - p1.position.y;
	double angle = atan2(dy, dx);
    return (angle);
}

double getDistBetweenPoses2D(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    double dx = p2.position.x - p1.position.x;
	double dy = p2.position.y - p1.position.y;
    return (sqrt(dx*dx+dy*dy));
}

void updatePose(const tf::StampedTransform &transform) {
	g_currentPose.position.x = transform.getOrigin().x();
	g_currentPose.position.y = transform.getOrigin().y();
	g_currentPose.position.z = transform.getOrigin().z();
	tf::quaternionTFToMsg(transform.getRotation(), g_currentPose.orientation);
}

geometry_msgs::Pose make2DPose(double x, double y) {
	geometry_msgs::Pose p;
	p.position.x = x;
	p.position.y = y;
	p.position.z = 0;
	return p;
}

std::vector<geometry_msgs::Pose> getCircleIntersectionsByY(geometry_msgs::Pose waypoint, geometry_msgs::Pose nextWaypoint, geometry_msgs::Pose robot, double distance) {
	double rpx = robot.position.x;
	double rpy = robot.position.y;

	double wx1 = waypoint.position.x;
	double wy1 = waypoint.position.y;

	double wx2 = nextWaypoint.position.x;
	double wy2 = nextWaypoint.position.y;

	
	double a = 1;	
	double b = -2*rpy;
	double c = rpy*rpy + wx1*wx1 - 2*rpx*wx1 + rpx*rpx - distance*distance;
	double D = (b*b - 4*a*c);

	if (D < 0.0) {
		return std::vector<geometry_msgs::Pose>();
	}
	
	double y1 = (-b + sqrt(D))/(2*a);
	double y2 = (-b - sqrt(D))/(2*a);

	if (D == 0.0) {
		std::vector<geometry_msgs::Pose> points {
			make2DPose(wx1, y1)
		};
		return points;
	}

	std::vector<geometry_msgs::Pose> points {
		make2DPose(wx1, y1),
		make2DPose(wx1, y2)
	};

	return points;
}

std::vector<geometry_msgs::Pose> getCircleIntersectionsByX(geometry_msgs::Pose waypoint, geometry_msgs::Pose nextWaypoint, geometry_msgs::Pose robot, double distance) {
	double rpx = robot.position.x;
	double rpy = robot.position.y;

	double wx1 = waypoint.position.x;
	double wy1 = waypoint.position.y;

	double wx2 = nextWaypoint.position.x;
	double wy2 = nextWaypoint.position.y;

	double dx = wx2 - wx1;
	double dy = wy2 - wy1;
	
	double a_l = dx == 0.0 ? 0.0 : dy/dx;
	double b_l = wy1 - a_l * wx1;
		
	double a = a_l*a_l + 1;	
	double b = -2*rpx + 2*a_l*(b_l-rpy);
	double c = rpx*rpx + (b_l - rpy)*(b_l - rpy) - distance*distance;
	double D = (b*b - 4*a*c);

	if (D < 0.0) {
		return std::vector<geometry_msgs::Pose>();
	}
	
	double x1 = (-b + sqrt(D))/(2*a);
	double x2 = (-b - sqrt(D))/(2*a);
	
	double y1 = a_l * x1 + b_l;
	double y2 = a_l * x2 + b_l;

	if (D == 0.0) {
		std::vector<geometry_msgs::Pose> points {
			make2DPose(x1, y1)
		};
		return points;
	}

	std::vector<geometry_msgs::Pose> points {
		make2DPose(x1, y1),
		make2DPose(x2, y2)
	};
	
	return points;
}
std::vector<geometry_msgs::Pose> getCircleIntersections(geometry_msgs::Pose waypoint, geometry_msgs::Pose nextWaypoint, geometry_msgs::Pose robot, double distance) {
	std::vector<geometry_msgs::Pose> intersections;

	double diff = waypoint.position.x - nextWaypoint.position.x;
	if(fabs(diff) < 0.01) {
		intersections = getCircleIntersectionsByY(waypoint, nextWaypoint, robot, distance);
	} else {
		intersections = getCircleIntersectionsByX(waypoint, nextWaypoint, robot, distance);
	}
	return intersections;
}

void cbPath(const nav_msgs::Path::ConstPtr &msg) {
	g_goals = msg->poses;
	ROS_INFO("Path sz: %d", (int)g_goals.size());
	for (auto goal : g_goals) {
		geometry_msgs::PoseStamped::ConstPtr goalPtr(new geometry_msgs::PoseStamped(goal));
	}
}

// there are only two points at most in points;
geometry_msgs::Pose getClosest(geometry_msgs::Pose goal, std::vector<geometry_msgs::Pose> points) {
	if (points.size() == 1) return points[0];
	double closestDistance = 1337;
	geometry_msgs::Pose closestPose;
	for (auto point : points) {
		double dist = getDistBetweenPoses2D(goal, point);
		if (dist < closestDistance) {
			closestPose = point;
			closestDistance = dist;
		}
	}
	return closestPose;
}

double getAngular(double dest_angle, double curr_angle, double gain){
	double error = (dest_angle - curr_angle);
	// account for how angles are reported. If the error is too big, we add or substract
	// 2pi to get the smallest error.
	if (error > M_PI) {
		error -= 2.0*M_PI;
	}
	if (error < -M_PI) {
		error += 2.0*M_PI;
	}
	return gain * error;
}

double findPerpendicularDistance(float multiplier) {	
	double wx1 = g_goals[0].pose.position.x;
	double wy1 = g_goals[0].pose.position.y;

	double wx2 = g_goals[1].pose.position.x;
	double wy2 = g_goals[1].pose.position.y;

	double px = g_currentPose.position.x;
	double py = g_currentPose.position.y;

	// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	double distance = fabs((wy2-wy1)*px - (wx2-wx1)*py + wx2*wy1 - wy2*wx1)/sqrt(pow(wy2-wy1,2.0)+pow(wx2-wx1,2.0));
	return distance * multiplier;
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void moveToNextPosition() {
	auto intersectionsToNextPoint = getCircleIntersections(g_goals[1].pose, g_goals[2].pose, g_currentPose, g_lookAhead);

	if(intersectionsToNextPoint.size() > 0) {
		// Always keep two points to create a line the robot can follow!
		if (g_goals.size() > 2 && getDistBetweenPoses2D(g_currentPose, g_goals[1].pose) < g_lookAhead) {
			g_goals.erase(g_goals.begin());
			ROS_INFO("Removed g_goals[0], goals left: %d", (int)g_goals.size());
			ROS_INFO("Intersections: %d", (int)intersectionsToNextPoint.size());
		}
	}

	auto intersections = getCircleIntersections(g_goals[0].pose, g_goals[1].pose, g_currentPose, g_lookAhead);

	if (intersections.size() == 0) {
		ROS_WARN("No intersections! Calculating shortest point to path");
		double newDist = findPerpendicularDistance(1.1);
		ROS_WARN("newDist: %lf", newDist);
		intersections = getCircleIntersections(g_goals[0].pose, g_goals[1].pose, g_currentPose, newDist);
		ROS_WARN("New intersects: %d", (int)intersections.size());
		if (intersections.size() == 0) ROS_ERROR("Can't find intersections");
	}
	
	geometry_msgs::Pose closestToGoal = getClosest(g_goals[1].pose, intersections);

	double distToNextGoal = getDistBetweenPoses2D(g_currentPose, g_goals[1].pose);
	ROS_DEBUG("Dist: %lf", distToNextGoal);

	if (g_goals.size() == 2 && distToNextGoal < g_tolerance) {
		g_goals.clear();
		ROS_INFO("Done navigating!");
		geometry_msgs::Twist vel_msg_stop;
		g_velPublisher.publish(vel_msg_stop);
		return;
	}

	// robot move to closestToGoal
	geometry_msgs::Twist vel_msg;

	
	double deltaAngle = getAngleBetweenPoses2D(g_currentPose, closestToGoal);	
	double currAngle = tf::getYaw(g_currentPose.orientation);

	double turnSpeed = getAngular(deltaAngle, currAngle, g_gain);
	if (distToNextGoal < g_lookAhead) {
		vel_msg.linear.x = fmin(distToNextGoal, g_maxSpeed);
	}
	else {
		double tmpSpd = map(M_PI - fabs(turnSpeed/g_gain), 0, M_PI, 0.0, 1.0);
		tmpSpd = pow(tmpSpd, g_pow);
		vel_msg.linear.x = tmpSpd * g_maxSpeed;
	}
	ROS_DEBUG("Vel: %lf", vel_msg.linear.x);

	vel_msg.angular.z = turnSpeed;

	ROS_DEBUG("Goal: %lf, %lf", closestToGoal.position.x,  closestToGoal.position.y);
	ROS_DEBUG("dA: %lf, A: %lf", deltaAngle, currAngle);
	ROS_DEBUG("turnSpeed: %lf", turnSpeed);

	g_velPublisher.publish(vel_msg);	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "follow_carrot_node");
	ros::NodeHandle n;
	
	ros::Subscriber subPath = n.subscribe("/plan", 100, cbPath);
	g_velPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	ros::Rate rate(100.0);
	tf::TransformListener listener;
	
	while (n.ok()) {
		tf::StampedTransform transform;
		try {
			listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			updatePose(transform);
			if (g_goals.size() > 0) {
				moveToNextPosition();			
			}
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("houdoe");
	//ros::spin();
	return 0;
}
