//============================================================================
// Name        : EposController.cpp
// Author      : Julian Stiefel
// Version     : 1.0.0
// Created on  : 26.04.2018
// Copyright   : BSD 3-Clause
// Description : Class providing the control and ROS interface for Maxon EPOS2
//				 (subscribers, parameters, timers, etc.).
//		 		 Install EPOS2 Linux Library from Maxon first!
//============================================================================

#include "maxon_epos2/EposController.hpp"

namespace maxon_epos2 {
EposController::EposController()
{
//   unsigned short id_list[2] = {7, 2};
	//Initialize device:
	if((epos_device_.initialization(id_list, motors))==MMC_FAILED) ROS_ERROR("Device initialization");
	//Start position mode during homing callback function:
	if((epos_device_.startPositionMode())==MMC_FAILED) ROS_ERROR("Starting position mode failed");
	//   if((epos_device_.setPositionProfile())==MMC_FAILED) ROS_ERROR("Seting position profile failed");
}

EposController::EposController(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	//   unsigned short id_list[2] = {7, 2};
	//Initialize device:
	if((epos_device_.initialization(id_list, motors))==MMC_FAILED) ROS_ERROR("Device initialization");
	//Start position mode during homing callback function:
	if((epos_device_.startPositionMode())==MMC_FAILED) ROS_ERROR("Starting position mode failed");
	//   if((epos_device_.setPositionProfile())==MMC_FAILED) ROS_ERROR("Seting position profile failed");


	publisher_ = nodeHandle_.advertise<maxon_epos2::epos_motor_info>(publisherTopic_, 10);
	homing_service_ = nodeHandle_.advertiseService("epos_homing_service", &EposController::homingCallback, this);
	service_ = nodeHandle_.advertiseService(serviceName_, &EposController::serviceCallback, this);


	ROS_INFO("Successfully launched EPOS Controller node.");
}

EposController::~EposController()
{
}

bool EposController::deviceOpenedCheck()
{
	return epos_device_.deviceOpenedCheck() == MMC_SUCCESS;
}

bool EposController::read(int id, float& pos, float& vel, float& eff)
{
	if(epos_device_.getPosition(id, &pos) == MMC_FAILED)
	{
		ROS_ERROR("Get position failed");
		return false;
	}
	if(epos_device_.getVelocity(id, &vel) == MMC_FAILED)
	{
		ROS_ERROR("Get velocity failed");
		return false;
	}
	eff = 0;
	return true;
	// if((epos_device_.deviceOpenedCheck()) == MMC_SUCCESS)
	// {
	// 	for(int i = 0; i < motors; i++)
	// 	{
	// 		epos_device_.getPosition(id_list[i], &poss[i]);
	// 		epos_device_.getVelocity(id_list[i], &vels[i]);
	// 		effs[i] = 0;
	// 	}
	// }
	// else
	// 	return false;
	// return true;
}

bool EposController::write(int id, float& cmd, float& vel)
{
	if(epos_device_.setPositionProfile(id, vel, 2 * vel, 2 * vel)==MMC_FAILED)
	{
		ROS_ERROR("Seting position profile failed");
		return false;
	}
	if(epos_device_.setPosition(id, cmd)==MMC_FAILED)
	{
		ROS_ERROR("Seting position failed");
		return false;
	}
	return true;
	// for(int i = 0; i < motors; i++)
	// {
	// 	if(epos_device_.setPositionProfile(id_list[i], vel, 2 * vel, 2 * vel)==MMC_FAILED)
	// 	{
	// 		ROS_ERROR("Seting position profile failed");
	// 		return false;
	// 	}
	// 	if(epos_device_.setPosition(id_list[i], cmds[i])==MMC_FAILED)
	// 	{
	// 		ROS_ERROR("setPosition failed");
	// 		return false;
	// 	}
	// }
	// return true;
}
bool EposController::readParameters()
{
	if (!nodeHandle_.getParam("publisher_topic", publisherTopic_)) return false;
	if (!nodeHandle_.getParam("service_name", serviceName_)) return false;
	return true;
}

bool EposController::homingCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
	ROS_INFO("Requested homing service");
	//Home device:
	ROS_INFO("Homing...");
	if((epos_device_.homing())==MMC_FAILED) ROS_ERROR("Device homing failed");
	else{
		//Start position mode:
		ROS_INFO("Start position mode");
		if((epos_device_.startPositionMode())==MMC_FAILED) ROS_ERROR("Starting position mode failed");
		response.success = MMC_SUCCESS;
	}
	return true;
}

bool EposController::serviceCallback(maxon_epos2::epos_motor_service::Request& request, maxon_epos2::epos_motor_service::Response& response){
	ROS_INFO_STREAM("Requested position" << request.position_setpoint[0]);
	for(int i = 0; i < request.motor_id.size(); i++)
	{
		if(epos_device_.setPositionProfile(request.motor_id[i], request.velocity[i], 2 * request.velocity[i], 2 * request.velocity[i])==MMC_FAILED) 
			ROS_ERROR("Seting position profile failed");
		if(epos_device_.setPosition(request.motor_id[i], request.position_setpoint[i])==MMC_FAILED) 
			ROS_ERROR("setPosition failed");
	}
	response.success = true;
	// if((epos_device_.getPosition(0, &response.position)) == MMC_FAILED) ROS_ERROR("getPosition failed for service");
	// if((epos_device_.getVelocity(0, &response.velocity)) == MMC_FAILED) ROS_ERROR("getVelocity failed for service");
	return true;
}

void EposController::publisherLoop(){
	float position, velocity;
	maxon_epos2::epos_motor_info motor;
	if((epos_device_.deviceOpenedCheck()) == MMC_SUCCESS)
	{
		for(int i = 0; i < sizeof(id_list)/sizeof(short); i++)
		{
			epos_device_.getPosition(id_list[i], &position);
			epos_device_.getVelocity(id_list[i], &velocity);
			motor.position.push_back(position);
			motor.velocity.push_back(velocity);
			motor.motor_id.push_back(id_list[i]);
		}
		
// ****only output these for DEBUGGING******
//		if((epos_device_.getPosition(&motor.position)) == MMC_FAILED) ROS_ERROR("getPosition failed for message");
//		if((epos_device_.getVelocity(&motor.velocity)) == MMC_FAILED) ROS_ERROR("getVelocity failed for message");
		publisher_.publish(motor);
	}
	else{
		//****only for DEBUGGING****
//		ROS_INFO("Nothing to publish.");
	}
}

void EposController::closeDevice(){
	  if((epos_device_.closeDevice()) == MMC_FAILED) ROS_ERROR("Device closing failed");
}

} /* namespace */
