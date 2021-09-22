#pragma once

#include <cstdio>
#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <controller_manager/controller_manager.h>

#include "manipulator/hardware_interface.h"
#include "manipulator/joint_data.h"


#define DOF 7

enum ArmState {Idle, Busy, Error, Disable};

class Manipulator
{
private:
    void jointDataInit();
    /* data */
    ArmState arm_state;
    hardware_interface::BlueArmInterface* blue_arm_interface;
    controller_manager::ControllerManager* blue_arm_cm;

    ros::NodeHandle& nodeHandle_;

public:
    Manipulator(ros::NodeHandle& nodeHandle);
    ~Manipulator();
    void closeDevice();
    void process(ros::Rate& loop_rate);
    std::vector<JointData*> joint_data;
    float sample_rate;
    std::string control_mode;
};
