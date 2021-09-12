#pragma once

#include <cstdio>
#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <controller_manager/controller_manager.h>

#include "maxon_epos2/epos_motor_info.h"
#include "maxon_epos2/epos_motor_service.h"
#include "manipulator/hardware_interface.h"
#include "manipulator/joint_data.h"
#include "manipulator/ArmState.h"
#include "manipulator/JointMove.h"
#include "manipulator/LineMove.h"
#include "manipulator/P2PMove.h"


#define DOF 7

enum ArmState {Idle, Busy, Error, Disable};

class Manipulator
{
private:
    void joint_data_init();
    /* data */
    ArmState arm_state;
    hardware_interface::BlueArmInterface* blue_arm_interface;
    controller_manager::ControllerManager* blue_arm_cm;

    ros::NodeHandle& nodeHandle_;

    // //! ROS topic Pub.
    // ros::Publisher arm_state_pub;

    // //! ROS topic Sub.
    // ros::Subscriber joint_state_sub;

    // //! ROS service server
    // ros::ServiceServer joint_move_server;
    // ros::ServiceServer p2p_move_server;
    // ros::ServiceServer line_move_server;

    // //! ROS service client
    // ros::ServiceClient motor_cmd_client;



public:
    Manipulator(ros::NodeHandle& nodeHandle);
    // Manipulator(ros::NodeHandle& nodeHandle);
    ~Manipulator();
    void joint_state_cb(const maxon_epos2::epos_motor_info::ConstPtr &msg);
    bool joint_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res);
    bool p2p_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res);
    bool line_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res);
    bool kinematics();
    void statePublisher();
    void closeDevice();
    void process(ros::Rate& loop_rate);
    std::vector<JointData*> joint_data;
    float sample_rate;
};