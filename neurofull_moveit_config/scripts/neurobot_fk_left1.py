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

def moveit():
    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
    
    #rospy.init_node('moveit_ik')
    #rospy.Subscriber("chatter", Pose, callback)      
    # Initialize the move group for the left arm
    left_arm = moveit_commander.MoveGroupCommander('left_arm')
    left_gripper = moveit_commander.MoveGroupCommander('left_gripper')
    #left_arm.set_end_effector_link('left_arm_link_6')
                  
    # Get the name of the end-effector link
    left_eef= left_arm.get_end_effector_link()
                    
    # Set the reference frame for pose targets
    reference_frame = 'base_footprint'
    
    # Set the left arm reference frame accordingly
    left_arm.set_pose_reference_frame(reference_frame)
            
    # Allow replanning to increase the odds of a solution
    left_arm.allow_replanning(True)
    
    # Allow some leeway in position (meters) and orientation (radians)
    left_arm.set_goal_position_tolerance(0.01)
    left_arm.set_goal_orientation_tolerance(0.01)
    #left_arm.set_named_target('left_arm_init')
    #left_arm.go()
    
  
    joint_positions = left_arm.get_current_joint_values()
       
    joint_positions1=[0,0,0,0,1.57,0]
    left_arm.set_joint_value_target(joint_positions1)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(1)
    joint_positions1=[-0.4,0,0,1.57,1.57,0.4]
    left_arm.set_joint_value_target(joint_positions1)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(3)
    left_gripper.set_joint_value_target(left_gripper_open)
    left_gripper.go()
    rospy.sleep(1)
    joint_positions1=[0,0,0,1.57,1.57,0]
    left_arm.set_joint_value_target(joint_positions1)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(3)    

    joint_positions = left_arm.get_current_joint_values()
    joint_positions1=[0.4,0.1,0,0.65,1.57,0.5]
    left_arm.set_joint_value_target(joint_positions1)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(2)

    left_gripper.set_joint_value_target(left_gripper_close)
    left_gripper.go()
    rospy.sleep(2)

    joint_positions1=[0.47,0,0,1.2,1.57,0.43]
    left_arm.set_joint_value_target(joint_positions1)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(0.5)
    joint_positions1=[0,0,0,1.57,1.57,0]
    left_arm.set_joint_value_target(joint_positions1)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(1)

    # Shut down MoveIt cleanly
    #moveit_commander.roscpp_shutdown()
    
    # Exit MoveIt
    moveit_commander.os._exit(0)

def hhh():
    rospy.init_node('moveit_yuyinonee')
    moveit()
    rospy.spin()

if __name__ == "__main__":
    try:
        hhh()
    except KeyboardInterrupt:
        raise

    
    
