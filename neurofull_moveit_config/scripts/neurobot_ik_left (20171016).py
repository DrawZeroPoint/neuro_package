#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

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
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from copy import deepcopy
REFERENCE_FRAME = 'base_footprint'

left_gripper_open = [1.57]
left_gripper_close = [0]
global a,b,c,d,e,f,g

def callback(data):
      global a1,b1,c1,d1,e1,f1,g1
      a1=data.pose.position.x
      b1=data.pose.position.y
      c1=data.pose.position.z
      d1=data.pose.orientation.x
      e1=data.pose.orientation.y
      f1=data.pose.orientation.z
      g1=data.pose.orientation.w
      q=MoveItDemo(a1,b1,c1,d1,e1,f1,g1)

class MoveItDemo:
   
    # spin() simply keeps python from exiting until this node is stopped

    def __init__(self,a,b,c,d,e,f,g):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
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
        left_arm.set_named_target('left_arm_init')
        left_arm.go()

        # Start the arm in the "resting" pose stored in the SRDF file
        joint_positions = left_arm.get_current_joint_values()
           
        joint_positions1=[0,0,0,0,1.57,0]
        left_arm.set_joint_value_target(joint_positions1)
        traj = left_arm.plan()
        left_arm.execute(traj)
        rospy.sleep(1)
        joint_positions = left_arm.get_current_joint_values()
        joint_positions1=[-0.4,0,0,1.57,1.57,0.4]
        left_arm.set_joint_value_target(joint_positions1)
        traj = left_arm.plan()
        left_arm.execute(traj)
        rospy.sleep(1)
        left_gripper.set_joint_value_target(left_gripper_open)
        left_gripper.go()
        rospy.sleep(1)
        joint_positions = left_arm.get_current_joint_values()
        joint_positions1=[0,0,0,1.57,1.57,0]
        left_arm.set_joint_value_target(joint_positions1)
        traj = left_arm.plan()
        left_arm.execute(traj)
        rospy.sleep(3)     
        # Set the target pose.  This particular pose has the gripper oriented horizontally
        # 0.85 meters above the ground, 0.10 meters to the left and 0.20 meters ahead of 
        # the center of the robot base.
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        
        target_pose.pose.position.x = a
        target_pose.pose.position.y = b-0.02
        target_pose.pose.position.z = c+0.43
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1
        
        #Set the start state to the current state
        left_arm.set_start_state_to_current_state()

        joint_positions_before = left_arm.get_current_joint_values()
        # Set the goal pose of the end effector to the stored pose
        left_arm.set_pose_target(target_pose, left_eef)
        
        # Plan the trajectory to the goal
        traj = left_arm.plan()
        n_joints = len(traj.joint_trajectory.joint_names)
        n_points = len(traj.joint_trajectory.points)

        #if joint_positions_before == traj.joint_trajectory.points[0].positions:
        if n_points==0:
            print '++++++++++++++++++++++++++'

            left_arm.set_named_target('left_arm_pose1')
            left_arm.go()
            rospy.sleep(0.5)
            left_arm.set_named_target('left_arm_init')
            left_arm.go()

            return
        # Execute the planned trajectory
            
        else:
            left_arm.execute(traj)
            rospy.sleep(2)
        # Pause for a second
        rospy.sleep(2)

        
        left_gripper.set_joint_value_target(left_gripper_close)
        left_gripper.go()
        rospy.sleep(2)
        joint_positions = left_arm.get_current_joint_values()

        #rint joint_positions[5]
        joint_positions1=[joint_positions[0],joint_positions[1],0,1.3,joint_positions[4],joint_positions[5]]
        left_arm.set_joint_value_target(joint_positions1)
        traj = left_arm.Splan()
        left_arm.execute(traj)
        rospy.sleep(2)
        joint_positions1=[0,0,0,1.57,1.57,0]
        left_arm.set_joint_value_target(joint_positions1)
        traj = left_arm.plan()
        left_arm.execute(traj)
        rospy.sleep(1)
        # Finish up in the resting position  
        left_arm.set_named_target('left_arm_pose1')
        left_arm.go()
        left_arm.set_named_target('left_arm_init')
        left_arm.go()

        # Shut down MoveIt cleanly
        #moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)
    
        # Set the color of an object
   

def ggg():
    rospy.init_node('moveit_ik_demo')
    rospy.Subscriber('/vision/grasp/pose',PoseStamped,callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        ggg()
    except KeyboardInterrupt:
        raise

    
    
