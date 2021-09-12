// #include <ros/ros.h>
#include "manipulator/Manipulator.h"
#include "maxon_epos2/EposController.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "blue_arm_control");
	ros::AsyncSpinner spinner(1);
    spinner.start();
	ros::NodeHandle nodeHandle;

	Manipulator blue_arm(nodeHandle);
	ros::Rate loop_rate(blue_arm.sample_rate);
	while (ros::ok()){
		blue_arm.process(loop_rate);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	spinner.stop();
	return 0;
}