#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector <move_base_msgs::MoveBaseGoal> createNavPoints() {
    std::vector <move_base_msgs::MoveBaseGoal> goals;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.w = 1;
    goals.push_back(goal);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 2.0;
    goal.target_pose.pose.position.y = 0.0;
    goals.push_back(goal);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 2.0;
    goal.target_pose.pose.position.y = 1.0;
    goals.push_back(goal);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 2.0;
    goal.target_pose.pose.position.y = 2.0;
    goals.push_back(goal);
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goals.push_back(goal);
    return goals;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    std::vector <move_base_msgs::MoveBaseGoal> goals = createNavPoints();

    ROS_INFO("Sending goal");
    ac.sendGoal(goals[0]);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray");
        ac.sendGoal(goals[1]);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Hooray");
            ac.sendGoal(goals[2]);

            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Hooray");
            }
        }


    }
    else {
        ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
    return 0;

}
