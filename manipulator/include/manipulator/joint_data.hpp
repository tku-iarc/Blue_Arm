#pragma once

#include <cstdio>
#include <iostream>
#include <string>

struct JointData
{
public:

    int id_;
    string joint_name_;
    float joint_angle_;
    float min_angle_;
    float max_angle_;
    float max_velocity_;
    float velocity_;
    float acceleration_;
    float deceleration_;
    float angle_cmd_;
    float velocity_cmd_;
    float effort_;
};