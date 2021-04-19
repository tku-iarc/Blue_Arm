#!/usr/bin/env python

import os
import sys
import rospy
from manipulator.msg import ArmState
from manipulator.srv import JointMove, JointMoveRequest
arm_state = 3
task_state = 0
joint_angles = [[0.425142765045166, -0.14600428938865662, -0.002285631373524666, -0.40503227710723877, 0.03209087625145912, -0.273002564907074, 0.0],
               [0.4237315058708191, -0.572803795337677, 0.754243016242981, -0.9078404903411865, 0.4782952070236206, -0.15741710364818573, 0.0],
               [0.1777883768081665, -0.9783115983009338, -0.20398876070976257, -1.8186415433883667, -0.1565580815076828, 0.2420785427093506, 0.0],
               [-0.5965037941932678, -1.4036997556686401, -0.06019340455532074, -2.8675010204315186, -0.24476197361946106, -0.30156486928462982, 0.0],
               [-0.9337647557258606, -2.638983964920044, -0.05844466760754585, -2.469801187515259, 0.02382272109389305, -0.10572195798158646, 0.0],
               [-0.9264169931411743, -1.0111234188079834, 0.7314173579216003, -0.726830780506134, 0.24819809198379517, -0.6670055389404297, 0.0],
               [0.5445324778556824, -2.7175543308258057, 0.23971517384052277, -2.862086057662964, -0.2559753656387329, -0.3285473819822073, 0.0]]
def armStateCallback(data):
    global arm_state
    arm_state = data.state

def joint_move_client(cmd):
    rospy.wait_for_service('/manipulator/joint_move')
    try:
        joint_move = rospy.ServiceProxy('/manipulator/joint_move', JointMove)
        success = joint_move(cmd)
        return success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    
    rospy.init_node('demo_task', anonymous=True)
    rospy.Subscriber("/manipulator/arm_state_msg", ArmState, armStateCallback)
    rate = rospy.Rate(10)
    arm_move = False
    while not rospy.is_shutdown():
        rate.sleep()
        if arm_move and arm_state == 1:
            arm_move = False
        if arm_state != 0 or arm_move:
            continue
        cmd = JointMoveRequest()
        cmd.joint_angle = joint_angles[task_state]
        joint_move_client(cmd)
        task_state = 0 if task_state is 6 else task_state + 1
        arm_move = True
        



