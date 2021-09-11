#include "manipulator/Manipulator.h"

Manipulator::Manipulator()
{
    sample_rate = 100;
    arm_state = Disable;
    joint_data_init();
    blue_arm_interface = new hardware_interface::BlueArmInterface(this->joint_data, sample_rate);
    blue_arm_cm = new controller_manager::ControllerManager(blue_arm_interface);
    bool load_controller = blue_arm_cm->loadController("blue_arm_controller");
    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!! "<<load_controller<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
}

// Manipulator::Manipulator(ros::NodeHandle& nodeHandle)
//     :nodeHandle_(nodeHandle)
// {
//     sample_rate = 100;
//     arm_state = Disable;
//     joint_data_init();
//     //! ROS topic Pub.
//     arm_state_pub = nodeHandle_.advertise<manipulator::ArmState>("/manipulator/arm_state_msg", 0);

//     //! ROS topic Sub.
//     joint_state_sub = nodeHandle_.subscribe("/maxon/epos_info_topic", 5, &Manipulator::joint_state_cb, this);

//     //! ROS service server
//     joint_move_server = nodeHandle_.advertiseService("/manipulator/joint_move", &Manipulator::joint_move_cb, this);
//     p2p_move_server = nodeHandle_.advertiseService("/manipulator/p2p_move", &Manipulator::p2p_move_cb, this);
//     line_move_server = nodeHandle_.advertiseService("/manipulator/line_move", &Manipulator::line_move_cb, this);

//     //! ROS service client
//     motor_cmd_client = nodeHandle_.serviceClient<maxon_epos2::epos_motor_service>("/maxon/epos_control_service", 0);
// }

Manipulator::~Manipulator()
{

}

void Manipulator::joint_data_init()
{
    joint_data.push_back(new JointData());
    joint_data[0]->id_           = 1;
    joint_data[0]->joint_name_   = "joint_1";
    joint_data[0]->joint_angle_  = 0;
    joint_data[0]->min_angle_    = -1 * M_PI;
    joint_data[0]->max_angle_    = M_PI;
    joint_data[0]->max_velocity_ = M_PI / 12;
    joint_data[0]->velocity_     = 0;
    joint_data[0]->acceleration_ = 0;
    joint_data[0]->deceleration_ = 0;
    joint_data[0]->angle_cmd_    = 0;
    joint_data[0]->velocity_cmd_ = 0;
    joint_data[0]->effort_       = 0;

    joint_data.push_back(new JointData());
    joint_data[1]->id_           = 2;
    joint_data[1]->joint_name_   = "joint_2";
    joint_data[1]->joint_angle_  = 0;
    joint_data[1]->min_angle_    = -1 * M_PI;
    joint_data[1]->max_angle_    = M_PI;
    joint_data[1]->max_velocity_ = M_PI / 12;
    joint_data[1]->velocity_     = 0;
    joint_data[1]->acceleration_ = 0;
    joint_data[1]->deceleration_ = 0;
    joint_data[1]->angle_cmd_    = 0;
    joint_data[1]->velocity_cmd_ = 0;
    joint_data[1]->effort_       = 0;

    joint_data.push_back(new JointData());
    joint_data[2]->id_           = 3;
    joint_data[2]->joint_name_   = "joint_3";
    joint_data[2]->joint_angle_  = 0;
    joint_data[2]->min_angle_    = -1 * M_PI;
    joint_data[2]->max_angle_    = M_PI;
    joint_data[2]->max_velocity_ = M_PI / 12;
    joint_data[2]->velocity_     = 0;
    joint_data[2]->acceleration_ = 0;
    joint_data[2]->deceleration_ = 0;
    joint_data[2]->angle_cmd_    = 0;
    joint_data[2]->velocity_cmd_ = 0;
    joint_data[2]->effort_       = 0;

    joint_data.push_back(new JointData());
    joint_data[3]->id_           = 4;
    joint_data[3]->joint_name_   = "joint_4";
    joint_data[3]->joint_angle_  = 0;
    joint_data[3]->min_angle_    = -1 * M_PI;
    joint_data[3]->max_angle_    = M_PI;
    joint_data[3]->max_velocity_ = M_PI / 12;
    joint_data[3]->velocity_     = 0;
    joint_data[3]->acceleration_ = 0;
    joint_data[3]->deceleration_ = 0;
    joint_data[3]->angle_cmd_    = 0;
    joint_data[3]->velocity_cmd_ = 0;
    joint_data[3]->effort_       = 0;

    joint_data.push_back(new JointData());
    joint_data[4]->id_           = 5;
    joint_data[4]->joint_name_   = "joint_5";
    joint_data[4]->joint_angle_  = 0;
    joint_data[4]->min_angle_    = -1 * M_PI;
    joint_data[4]->max_angle_    = M_PI;
    joint_data[4]->max_velocity_ = M_PI / 12;
    joint_data[4]->velocity_     = 0;
    joint_data[4]->acceleration_ = 0;
    joint_data[4]->deceleration_ = 0;
    joint_data[4]->angle_cmd_    = 0;
    joint_data[4]->velocity_cmd_ = 0;
    joint_data[4]->effort_       = 0;

    joint_data.push_back(new JointData());
    joint_data[5]->id_           = 6;
    joint_data[5]->joint_name_   = "joint_6";
    joint_data[5]->joint_angle_  = 0;
    joint_data[5]->min_angle_    = -1 * M_PI;
    joint_data[5]->max_angle_    = M_PI;
    joint_data[5]->max_velocity_ = M_PI / 12;
    joint_data[5]->velocity_     = 0;
    joint_data[5]->acceleration_ = 0;
    joint_data[5]->deceleration_ = 0;
    joint_data[5]->angle_cmd_    = 0;
    joint_data[5]->velocity_cmd_ = 0;
    joint_data[5]->effort_       = 0;

    joint_data.push_back(new JointData());
    joint_data[6]->id_           = 7;
    joint_data[6]->joint_name_   = "joint_7";
    joint_data[6]->joint_angle_  = 0;
    joint_data[6]->min_angle_    = -1 * M_PI;
    joint_data[6]->max_angle_    = M_PI;
    joint_data[6]->max_velocity_ = M_PI / 12;
    joint_data[6]->velocity_     = 0;
    joint_data[6]->acceleration_ = 0;
    joint_data[6]->deceleration_ = 0;
    joint_data[6]->angle_cmd_    = 0;
    joint_data[6]->velocity_cmd_ = 0;
    joint_data[6]->effort_       = 0;
}

void Manipulator::joint_state_cb(const maxon_epos2::epos_motor_info::ConstPtr &msg)
{
    float velocity_sum = 0;
    for(int i = 0; i < msg->motor_id.size(); i++)
    {
        if(joint_data[i]->id_ == msg->motor_id[i])
        {
            joint_data[i]->joint_angle_ = msg->position[i];
            joint_data[i]->velocity_ = msg->velocity[i];
        }
        velocity_sum += fabs(msg->velocity[i]);
    }
    arm_state = (velocity_sum < 0.01) ? Idle : Busy;
    return;
}

bool Manipulator::joint_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res)
{
    res.success = false;
    float move_dis[DOF];
    float max_move = 0;
    int max_move_axis = 0;
    float move_time = 2;
    for(int i = 0; i < req.joint_angle.size(); i++)
    {
        if(req.joint_angle[i] > joint_data[i]->max_angle_ || req.joint_angle[i] < joint_data[i]->min_angle_)
        {
            std::cout<<"FUCKKKKKKK"<<std::endl;
            return true;
        }
        joint_data[i]->angle_cmd_ = req.joint_angle[i];
        move_dis[i] = fabs(req.joint_angle[i] - joint_data[i]->joint_angle_);
        if(move_dis[i] > max_move)
        {
            max_move = move_dis[i];
            max_move_axis = i;
        }
    }
    move_time = (max_move / (move_time - 0.5) > joint_data[max_move_axis]->max_velocity_) ? (max_move / joint_data[max_move_axis]->max_velocity_) + 0.5 : move_time;
    std::cout<<"MOVE_TIME = "<<move_time<<std::endl;
    for(int i = 0; i < DOF; i++)
    {
        joint_data[i]->velocity_cmd_ = move_dis[i] / (move_time - 0.5);
    }
    maxon_epos2::epos_motor_service move_cmd;
   
    for(int i = 0; i < DOF; i++)
    {
        move_cmd.request.motor_id.push_back(joint_data[i]->id_);
        move_cmd.request.position_setpoint.push_back(joint_data[i]->angle_cmd_);
        move_cmd.request.velocity.push_back(joint_data[i]->velocity_cmd_);
        std::cout<<"velocity is "<<joint_data[i]->velocity_cmd_<<std::endl;
    }
    // if(motor_cmd_client.call(move_cmd))
    //     res.success = true;
    return true;
}
bool Manipulator::p2p_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res)
{

}
bool Manipulator::line_move_cb(manipulator::JointMove::Request &req, manipulator::JointMove::Response &res)
{

}
bool Manipulator::kinematics()
{

}
void Manipulator::statePublisher()
{
    manipulator::ArmState msg;
    msg.state = arm_state;
    // arm_state_pub.publish(msg);
}
void Manipulator::process()
{
    blue_arm_interface->read();
    blue_arm_cm->update(blue_arm_interface->get_time(), blue_arm_interface->get_period());
    blue_arm_interface->write();
}
