#pragma once
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "maxon_epos2/EposController.hpp"
#include "manipulator/joint_data.hpp"

class BlueArmInterface : public hardware_interface::RobotHW
{
public:
  BlueArmInterface(std::vector<JointData> &joint_data, float sample_rate)
  ~BlueArmInterface()
  bool read(void);
  bool write(void);
private:
  void checkCmdLimit(float& cmd, float& vel);

  float sample_rate;
  maxon_epos2::EposController epos_controller;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  std::vector<JointData> *jd_ptr;
  
};