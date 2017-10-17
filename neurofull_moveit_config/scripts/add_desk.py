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
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy
from std_msgs.msg import Float32MultiArray

gripper_open = [0]
gripper_close = [0]
#gripper_grab = [0.3]
class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_ik_demo_part')
        
        # Construct the initial scene object
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene,queue_size=100)
        trajectory_pos_pub = rospy.Publisher('my_trajectory_pos',Float32MultiArray,queue_size=10)
        trajectory_vel_pub = rospy.Publisher('my_trajectory_vel',Float32MultiArray,queue_size=10)
        # Create a dictionary to hold object colors
        self.colors = dict()
        
        # Pause for the scene to get ready
        rospy.sleep(1)
                        
        # Initialize the move group for the right arm
        left_arm = MoveGroupCommander('left_arm')
        left_gripper = MoveGroupCommander('left_gripper')
        # Get the name of the end-effector link
        left_eef = left_arm.get_end_effector_link()
        
        # Allow some leeway in position (meters) and orientation (radians)
        left_arm.set_goal_position_tolerance(0.005)
        left_arm.set_goal_orientation_tolerance(0.01)
       
        # Allow replanning to increase the odds of a solution
        left_arm.allow_replanning(True)
        
        reference_frame = left_arm.get_planning_frame()
        
        # Allow 5 seconds per planning attempt
        left_arm.set_planning_time(3)
        ####add table###
        table_id = 'table'
        table_ground = 0
        table_size = [0.42, 0.54, 0.4]
        scene.remove_world_object(table_id)
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.position.x = 0.75
        table_pose.pose.position.y = 0.25
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        self.setColor(table_id, 0.8, 0, 0.7, 1.0)

        # Start the arm in the "resting" pose stored in the SRDF file
        left_arm.set_named_target('left_init')
        left_arm.go()
        rospy.sleep(1)
        left_arm.set_named_target('pose1')
        left_arm.go()
        rospy.sleep(1)

        
        
        # Set the start state to the current state
        
        # Set the target pose in between the boxes and above the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp =rospy.Time.now()
        target_pose.pose.position.x = 0.45
        target_pose.pose.position.y = 0.25
        target_pose.pose.position.z = 0.45

        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1
        # Set the target pose for the arm
        left_arm.set_start_state_to_current_state()
        left_arm.set_pose_target(target_pose,left_eef)
        traj= left_arm.plan()
        left_arm.execute(traj)
        rospy.sleep(1)
        # Move the arm to the target pose (if possible)

        # Cartesian Paths
        

        
        
        #left_arm.set_named_target('left_init')
        #left_arm.go()
        #rospy.sleep(1)

        
        # Exit the script        
        moveit_commander.os._exit(0)
        
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    
        
       
if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise
    
    
    
