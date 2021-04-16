#!/usr/bin/env python

import os
import sys
import rospy
from manipulator.msg import ArmState
from manipulator.srv import JointMove
arm_state = 3
task_state = 0
joint_angles = [[0.,0.,0.,0.,0.,0.,0.],
               [0.,0.,0.,0.,0.,0.,0.]]
def armStateCallback(data):
    global arm_state
    arm_state = data.state

def joint_move_client(cmd):
    rospy.wait_for_service('manipulator/joint_move')
    try:
        joint_move = rospy.ServiceProxy('manipulator/joint_move', JointMove)
        success = joint_move(cmd)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    global arm_state, joint_angles
    rospy.init_node('demo_task', anonymous=True)
    rospy.Subscriber("manipulator/arm_state_msg", ArmState, armStateCallback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        if arm_state != 0:
            continue
        JointMove cmd
        cmd.request.joint_angle = joint_angles[task_state]
        task_state = 0 if task_state > 5 else task_state + 1



