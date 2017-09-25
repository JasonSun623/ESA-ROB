#include "ros/ros.h"


int main(int argc, char **argv) {
	ros::init(argc, argv, "servoing_node");
	ros::NodeHandle n;
    
    /*
     * More init
     */
    
	ros::spin();

	return 0;
}

