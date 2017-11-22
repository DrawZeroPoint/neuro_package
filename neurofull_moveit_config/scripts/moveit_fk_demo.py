#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1 2014-01-14
    
    Use forward kinemtatics to move the arm to a specified set of joint angles
    
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
from std_msgs.msg import Float32MultiArray

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)
        trajectory_pos_pub = rospy.Publisher('my_trajectory_pos',Float32MultiArray,queue_size=10)
        trajectory_vel_pub = rospy.Publisher('my_trajectory_vel',Float32MultiArray,queue_size=10)
        print "===== It is OK ===="
        rospy.sleep(3)
        
        # Connect to the left_arm move group
        left_arm = moveit_commander.MoveGroupCommander('left_arm') 
        # Set a small tolerance on joint angles
        left_arm.set_goal_joint_tolerance(0.01)
        
        joint_positions1 = [0,0,0,1.57,0,0]
        
        left_arm.set_joint_value_target(joint_positions1)
        traj = left_arm.plan()
        left_arm.execute(traj)
        rospy.sleep(2)
 
        traj_joint = Float32MultiArray()
        traj_joint.data = [0,0,0,0,0,0]       
        # Get the number of joints involved
        n_joints = len(traj.joint_trajectory.joint_names)
       
        # Get the number of points on the trajectory
        n_points = len(traj.joint_trajectory.points)
       
        # as well as joint speed and acceleration
        now_time = 0
        last_time = 0
        for i in range(n_points):
           
            # The joint positions are not scaled so pull them out first
            traj_joint.data = traj.joint_trajectory.points[i].positions
            trajectory_pos_pub.publish(traj_joint)
            traj_joint.data = traj.joint_trajectory.points[i].velocities
            trajectory_vel_pub.publish(traj_joint)
            now_time = traj.joint_trajectory.points[i].time_from_start.secs+traj.joint_trajectory.points[i].time_from_start.nsecs*1.0/1000000000
            tim = now_time-last_time
            last_time = now_time
            print tim
            rospy.sleep(tim)
        # Cleanly shut down MoveIt
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
