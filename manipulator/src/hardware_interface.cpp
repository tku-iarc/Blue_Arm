#include "manipulator/hardware_interface.h"
#include <ros/ros.h>

namespace hardware_interface
{

BlueArmInterface::BlueArmInterface(std::vector<JointData*>& joint_data, float sample_rate) 
{ 
    this->sample_rate = sample_rate;
    jd_ptr = joint_data;
    time_now = ros::Time::now();
    time_last = ros::Time::now();
    for (int i=0; i < jd_ptr.size(); i++)
    {   
        hardware_interface::JointStateHandle state_handle(jd_ptr[i]->joint_name_, &jd_ptr[i]->joint_angle_, &jd_ptr[i]->velocity_, &jd_ptr[i]->effort_);
        jnt_state_interface.registerHandle(state_handle);
        
    }
    // // connect and register the joint state interface
    // hardware_interface::JointStateHandle state_handle_1("joint_1", &poss[0], &vels[0], &effs[0]);
    // jnt_state_interface.registerHandle(state_handle_1);

    // hardware_interface::JointStateHandle state_handle_2("joint_2", &poss[1], &vels[1], &effs[1]);
    // jnt_state_interface.registerHandle(state_handle_2);

    // hardware_interface::JointStateHandle state_handle_3("joint_3", &poss[2], &vels[2], &effs[2]);
    // jnt_state_interface.registerHandle(state_handle_3);

    // hardware_interface::JointStateHandle state_handle_4("joint_4", &poss[3], &vels[3], &effs[3]);
    // jnt_state_interface.registerHandle(state_handle_4);

    // hardware_interface::JointStateHandle state_handle_5("joint_5", &poss[4], &vels[4], &effs[4]);
    // jnt_state_interface.registerHandle(state_handle_5);

    // hardware_interface::JointStateHandle state_handle_6("joint_6", &poss[5], &vels[5], &effs[5]);
    // jnt_state_interface.registerHandle(state_handle_6);

    // hardware_interface::JointStateHandle state_handle_7("joint_7", &poss[6], &vels[6], &effs[6]);
    // jnt_state_interface.registerHandle(state_handle_7);

    registerInterface(&jnt_state_interface);

    for (int i=0; i < jd_ptr.size(); i++)
    {
        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(jd_ptr[i]->joint_name_), &jd_ptr[i]->angle_cmd_);
        jnt_pos_interface.registerHandle(pos_handle);
    }
    // // connect and register the joint position interface
    // hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("joint_1"), &cmds[0]);
    // jnt_pos_interface.registerHandle(pos_handle_1);

    // hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("joint_2"), &cmds[1]);
    // jnt_pos_interface.registerHandle(pos_handle_2);

    // hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("joint_3"), &cmds[2]);
    // jnt_pos_interface.registerHandle(pos_handle_3);

    // hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("joint_4"), &cmds[3]);
    // jnt_pos_interface.registerHandle(pos_handle_4);

    // hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("joint_5"), &cmds[4]);
    // jnt_pos_interface.registerHandle(pos_handle_5);

    // hardware_interface::JointHandle pos_handle_6(jnt_state_interface.getHandle("joint_6"), &cmds[5]);
    // jnt_pos_interface.registerHandle(pos_handle_6);

    // hardware_interface::JointHandle pos_handle_7(jnt_state_interface.getHandle("joint_7"), &cmds[6]);
    // jnt_pos_interface.registerHandle(pos_handle_7);

    registerInterface(&jnt_pos_interface);
}

BlueArmInterface::~BlueArmInterface()
{
    //if node is interrupted, close device
    epos_controller.closeDevice();
}

void BlueArmInterface::checkCmdLimit(int cmd_indx)
{
    if(jd_ptr[cmd_indx]->velocity_cmd_ > jd_ptr[cmd_indx]->max_velocity_)
        jd_ptr[cmd_indx]->velocity_cmd_ = jd_ptr[cmd_indx]->max_velocity_;
    if(jd_ptr[cmd_indx]->angle_cmd_ > jd_ptr[cmd_indx]->max_angle_)
        jd_ptr[cmd_indx]->angle_cmd_ = jd_ptr[cmd_indx]->max_angle_;
    if(jd_ptr[cmd_indx]->angle_cmd_ < jd_ptr[cmd_indx]->min_angle_)
        jd_ptr[cmd_indx]->angle_cmd_ = jd_ptr[cmd_indx]->min_angle_;
}

bool BlueArmInterface::read()
{
    if(epos_controller.deviceOpenedCheck())
        return false;
    for (int i=0; i < jd_ptr.size(); i++)
    {
        if(epos_controller.read(jd_ptr[i]->id_, jd_ptr[i]->joint_angle_, jd_ptr[i]->velocity_, jd_ptr[i]->effort_) == false)
        {
            ROS_ERROR("Read Joint States Fail!!!");
            return false;
        }
    }
    return true;
}

bool BlueArmInterface::write()
{
    if(epos_controller.deviceOpenedCheck() == false)
        return false;
    for (int i=0; i < jd_ptr.size(); i++)
    {
        float move_dis = fabs(jd_ptr[i]->angle_cmd_ - jd_ptr[i]->joint_angle_);
        jd_ptr[i]->velocity_cmd_ = move_dis * sample_rate; // move_dis / (1 / sample_rate)
        checkCmdLimit(i);
        if(epos_controller.write(jd_ptr[i]->id_, jd_ptr[i]->angle_cmd_, jd_ptr[i]->velocity_cmd_) == false)
        {
            ROS_ERROR("Write Joint States Fail!!!");
            return false;
        }
    }
    return true;
}

ros::Time BlueArmInterface::get_time()
{
    time_now = ros::Time::now();
    return time_now;
}
ros::Duration BlueArmInterface::get_period()
{
    period = time_now - time_last;
    time_last = time_now;
    return period;
}

}