#pragma once
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "maxon_epos2/EposController.hpp"
#include "manipulator/joint_data.h"

namespace hardware_interface
{
  
class BlueArmInterface : public hardware_interface::RobotHW
{
public:
  BlueArmInterface(std::vector<JointData*>& joint_data, float sample_rate);
  ~BlueArmInterface();
  bool read(void);
  bool write(void);
  ros::Time get_time();
  ros::Duration get_period();
  float sample_rate;

private:
  void checkCmdLimit(int cmd_indx);
  maxon_epos2::EposController epos_controller;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  std::vector<JointData*> jd_ptr;
  ros::Time time_now;
  ros::Time time_last;
  ros::Duration period;
};

} // namespace name