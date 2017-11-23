#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyleft (c) 2014 Patrick Goebel.  All lefts reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

left_gripper_open = [1.57]
left_gripper_close = [0]


def callback(msg):
    forward = msg.data
    if forward == 3:
        moveit_forward()
    if forward == 4:
        moveit_backward()
    if forward == 5:
        moveit_left()
    if forward == 6:
        moveit_right()
        # spin() simply keeps python from exiting until this node is stopped


def moveit_forward():
    moveit_commander.roscpp_initialize(sys.argv)
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_gripper = moveit_commander.MoveGroupCommander('left_gripper')
    left_eef = left_arm.get_end_effector_link()
    reference_frame = 'base_footprint'
    left_arm.set_pose_reference_frame(reference_frame)
    left_arm.allow_replanning(True)
    left_arm.set_goal_position_tolerance(0.01)
    left_arm.set_goal_orientation_tolerance(0.01)
    left_arm.shift_pose_target(0, 0.05, left_eef)
    left_arm.go()
    rospy.sleep(1)


def moveit_backward():
    moveit_commander.roscpp_initialize(sys.argv)
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_gripper = moveit_commander.MoveGroupCommander('left_gripper')
    left_eef = left_arm.get_end_effector_link()
    reference_frame = 'base_footprint'
    left_arm.set_pose_reference_frame(reference_frame)
    left_arm.allow_replanning(True)
    left_arm.set_goal_position_tolerance(0.01)
    left_arm.set_goal_orientation_tolerance(0.01)
    left_arm.shift_pose_target(0, -0.05, left_eef)
    left_arm.go()
    rospy.sleep(1)


def moveit_left():
    moveit_commander.roscpp_initialize(sys.argv)
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_gripper = moveit_commander.MoveGroupCommander('left_gripper')
    left_eef = left_arm.get_end_effector_link()
    reference_frame = 'base_footprint'
    left_arm.set_pose_reference_frame(reference_frame)
    left_arm.allow_replanning(True)
    left_arm.set_goal_position_tolerance(0.01)
    left_arm.set_goal_orientation_tolerance(0.01)
    left_arm.shift_pose_target(1, 0.05, left_eef)
    left_arm.go()
    rospy.sleep(1)


def moveit_right():
    moveit_commander.roscpp_initialize(sys.argv)
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_gripper = moveit_commander.MoveGroupCommander('left_gripper')
    left_eef = left_arm.get_end_effector_link()
    reference_frame = 'base_footprint'
    left_arm.set_pose_reference_frame(reference_frame)
    left_arm.allow_replanning(True)
    left_arm.set_goal_position_tolerance(0.01)
    left_arm.set_goal_orientation_tolerance(0.01)
    left_arm.shift_pose_target(1, -0.05, left_eef)
    left_arm.go()
    rospy.sleep(1)


def init():
    rospy.init_node('moveit_arm_left_move')
    rospy.Subscriber('/call/leftarm', Int32, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        init()
    except KeyboardInterrupt:
        raise
