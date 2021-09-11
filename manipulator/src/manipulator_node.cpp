// #include <ros/ros.h>
#include "manipulator/Manipulator.h"
#include "maxon_epos2/EposController.hpp"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "blue_arm");
	ros::NodeHandle nodeHandle("~");

	if(true)
	{
		Manipulator blue_arm;
		ros::Rate loop_rate(blue_arm.sample_rate);
		while (ros::ok()){
			blue_arm.process();
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else
	{
		// Manipulator blue_arm(nodeHandle);
		// ros::Rate loop_rate(blue_arm.sample_rate);
		// while (ros::ok()){
		// 	blue_arm.statePublisher();
		// 	ros::spinOnce();
		// 	loop_rate.sleep();
		// }
	}
	
	return 0;
}