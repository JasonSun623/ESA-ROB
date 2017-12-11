#include "ros/ros.h"
#include "assignment5/Triangle.h"
#include <math.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

turtlebot_actions::TurtlebotMoveGoal createGoal(float forward_distance, float turn_distance) {
	turtlebot_actions::TurtlebotMoveGoal goal;
	goal.forward_distance = forward_distance;
	goal.turn_distance = turn_distance;
	return goal;
}

void doneCb(const actionlib::SimpleClientGoalState& state,
            const turtlebot_actions::TurtlebotMoveResultConstPtr& result) {
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

void activeCb() {
	ROS_INFO("Goal just went active");
}

void feedbackCb(const turtlebot_actions::TurtlebotMoveFeedbackConstPtr& feedback) {
	ROS_INFO("Dist: %f, Turn: %f ", feedback->forward_distance, feedback->turn_distance);
}

void cmdCallback(const assignment5::Triangle::ConstPtr& msg) 
{
	actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> ac("turtlebot_move", true);
	ROS_INFO("Waiting for server");
	ac.waitForServer();
	ROS_INFO("Done waiting for server");
	
	float cw = msg->cw ? -1.0f : 1.0f;
	for (int i = 0; i < 3; i++) {
		ac.sendGoal(createGoal(msg->sideLength, 0), &doneCb, &activeCb, &feedbackCb);
		ac.sendGoal(createGoal(0, cw * M_PI/3.0 * 2), &doneCb, &activeCb, &feedbackCb);
	}
	ROS_INFO("Goal sent");
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "move_triangle");	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd", 100, cmdCallback);
	ros::Rate loop_rate(100);
	ros::spin();
	return 0;
}
