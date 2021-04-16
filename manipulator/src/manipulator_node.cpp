// #include <ros/ros.h>
#include "manipulator/Manipulator.hpp"


int main(int argc, char** argv)
{
    std::cout<<"1"<<std::endl;
	ros::init(argc, argv, "blue_arm");
	ros::NodeHandle nodeHandle("~");
	ros::Rate loop_rate(10);

    std::cout<<"2"<<std::endl;

	Manipulator blue_arm(nodeHandle);
    
    std::cout<<"3"<<std::endl;
	//publish until node gets interrupted
	while (ros::ok()){
		blue_arm.process();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}