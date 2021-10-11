#!/usr/bin/env python
import rospy
from manipulator import BlueArmMoveGroup

def main():
    rospy.init_node('blue_arm_sample', anonymous=True)
    blue_arm = BlueArmMoveGroup()

    pos = [0.3, 0, 0.15]
    euler = [0, 0, 0]
    blue_arm.set_speed(0.8)
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")

    pos = [0.3, 0, 0.15]
    euler = [30, 0, 0]
    blue_arm.set_speed(0.6)
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")

    pos = [0.3, 0, 0.15]
    euler = [0, 30, 0]
    blue_arm.set_speed(0.4)
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")

    pos = [0.3, 0, 0.15]
    euler = [0, 0, 30]
    blue_arm.set_speed(0.2)
    if blue_arm.go_to_pose_goal(pos, euler) is False:
        rospy.logerr("Move Robot Failed!!")
     
if __name__ == '__main__':
    main()