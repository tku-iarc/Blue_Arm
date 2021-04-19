#pragma once

#include <cstdio>
#include <iostream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>

#include "maxon_epos2/epos_motor_info.h"
#include "maxon_epos2/epos_motor_service.h"
#include "manipulator/ArmState.h"
#include "manipulator/JointMove.h"
#include "manipulator/LineMove.h"
#include "manipulator/P2PMove.h"


#define DOF 7

enum ArmState {Idle, Busy, Error, Disable};

struct JointData
{
public:

    int id_;
    float joint_angle_;
    float min_angle_;
    float max_angle_;
    float max_velocity_;
    float velocity_;
    float acceleration_;
    float deceleration_;
    float angle_cmd_;
    float velocity_cmd_;
};

class Manipulator
{
private:
    /* data */
    ArmState arm_state;
    JointData joint_data[DOF];

    ros::NodeHandle& nodeHandle_;

    //! ROS topic Pub.
    ros::Publisher arm_state_pub;

    //! ROS topic Sub.
    ros::Subscriber joint_state_sub;

    //! ROS service server
    ros::ServiceServer joint_move_server;
    ros::ServiceServer p2p_move_server;
    ros::ServiceServer line_move_server;

    //! ROS service client
    ros::ServiceClient motor_cmd_client;

public:
    Manipulator(ros::NodeHandle& nodeHandle);
    ~Manipulator();
    void joint_state_cb(const maxon_epos2::epos_motor_info::ConstPtr &msg);
    bool joint_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res);
    bool p2p_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res);
    bool line_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res);
    bool kinematics();
    void process();
};
