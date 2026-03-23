/*
	FILE: ccmtracking_controller_node.cpp
	---------------------------------
	tracking controller for px4-based quadcopter
*/
#include <CCMtrackingController.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "ccm_tracking_controller_node");
	ros::NodeHandle nh;
	ccmtracking::CCMtrackingController tc (nh);
	ros::spin();

	return 0;
}