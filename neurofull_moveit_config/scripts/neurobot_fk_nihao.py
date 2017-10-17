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
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int32

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global aa
def callback(data):
      
      global a1
      a1=data
      q=MoveItDemo(a1)

class MoveItDemo:
   
    # spin() simply keeps python from exiting until this node is stopped
     
    def __init__(self,aa):
        #if aa==2:
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        #rospy.init_node('moveit_ik')
        #rospy.Subscriber("chatter", Pose, callback)      
        # Initialize the move group for the right arm
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
        #right_arm.set_end_effector_link('right_arm_link_6')
                      
        # Get the name of the end-effector link
        right_eef= right_arm.get_end_effector_link()
                        
        # Set the reference frame for pose targets
        reference_frame = 'base_footprint'
        
        # Set the right arm reference frame accordingly
        right_arm.set_pose_reference_frame(reference_frame)
                
        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Allow some leeway in position (meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.01)
        right_arm.set_goal_orientation_tolerance(0.01)

        right_arm.set_named_target('right_arm_init')
	right_arm.go()

	
        joint_positions = right_arm.get_current_joint_values()

        joint_positions1=[0,1.1,1.57,1.57,0,0]
        right_arm.set_joint_value_target(joint_positions1)
        traj = right_arm.plan()
        right_arm.execute(traj)
        rospy.sleep(6)
        
        # Start the arm in the "resting" pose stored in the SRDF file\  
        # Set the target pose.  This particular pose has the gripper oriented horizontally
        # 0.85 meters above the ground, 0.10 meters to the right and 0.20 meters ahead of 
        # the center of the robot base.
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        global a,b,c,d,e,f,g  
        target_pose.pose.position.x = 0.45
        target_pose.pose.position.y = -0.28
        target_pose.pose.position.z = 1
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 1
        #Set the start state to the current state
        right_arm.set_start_state_to_current_state()
        
        # Set the goal pose of the end effector to the stored pose
        right_arm.set_pose_target(target_pose, right_eef)
        
        # Plan the trajectory to the goal
        #traj = right_arm.plan()
        #print traj

        # Execute the planned trajectory
        #right_arm.execute(traj)
        
        # Pause for a second
        #rospy.sleep(2)

        #right_arm.shift_pose_target(0,0.02,right_eef)
        #right_arm.go()
        
        

        joint_positions = right_arm.get_current_joint_values()
        joint_positions1=[0,0,0,0,0,0]
        right_arm.set_joint_value_target(joint_positions1)
        traj = right_arm.plan()
        right_arm.execute(traj)
        rospy.sleep(2)
		

        #right_arm.shift_pose_target(2,0.01,right_eef)
        #right_arm.go()
        #rospy.sleep(1)
        # Finish up in the resting position  
        #right_arm.set_named_target('right_arm_pose1')
        #right_arm.go()

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit MoveIt
        moveit_commander.os._exit(0)

	
           
def hhh():
    rospy.init_node('moveit_yuyin')
    rospy.Subscriber('/call/hello',Int32,callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        hhh()
        q=MoveItDemo(a1)
    except KeyboardInterrupt:
        raise

    
    
