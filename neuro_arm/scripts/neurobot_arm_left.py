#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinematics to move the end effector to a specified pose
    
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

    global_name = rospy.get_param("/global_name")
    relative_name = rospy.get_param("relative_name")
    private_param = rospy.get_param('~private_name')
    default_param = rospy.get_param('default_param', 'default_value')
"""

import rospy
import sys
import moveit_commander
from moveit_commander import PlanningSceneInterface

from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped

left_gripper_open = [1.57]
left_gripper_close = [0]

# frame distance in z direction
link_to_foot_ = 0.465

# Initialize the move_group API
moveit_commander.roscpp_initialize(sys.argv)

rospy.sleep(8)  # Wait moveit to start up, necessary!

# Initialize the move group for the left arm
left_arm = moveit_commander.MoveGroupCommander('left_arm')
left_gripper = moveit_commander.MoveGroupCommander('left_gripper')

# Set the reference frame for pose targets
reference_frame = 'base_link'
# Set the left arm reference frame accordingly
left_arm.set_pose_reference_frame(reference_frame)

# Get the name of the end-effector link
left_eef = left_arm.get_end_effector_link()

# Allow re-planning to increase the odds of a solution
left_arm.allow_replanning(True)

# Allow some leeway in position (meters) and orientation (radians)
left_arm.set_goal_position_tolerance(0.01)
left_arm.set_goal_orientation_tolerance(0.01)

# Construct the initial scene object
scene = PlanningSceneInterface()
map_frame = 'map'  # Reference frame for scene

# Publish inverse kinetic result
ik_result_pub = rospy.Publisher('/feed/arm/left/ik/plan/result', Int8, queue_size=1)


def reset():
    # Define end effector pose, this pose make robot put down its arm
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()

    target_pose.pose.position.x = 0.45
    target_pose.pose.position.y = 0.28
    target_pose.pose.position.z = 1
    target_pose.pose.orientation.x = 0
    target_pose.pose.orientation.y = 0
    target_pose.pose.orientation.z = 0
    target_pose.pose.orientation.w = 1

    # Set the start state to the current state
    left_arm.set_start_state_to_current_state()

    # Set the goal pose of the end effector to the stored pose
    left_arm.set_pose_target(target_pose, left_eef)

    # Reset using inverse kinetic
    init_positions = [0, 0, 0, 0, 0, 0]
    left_arm.set_joint_value_target(init_positions)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(1)

    # Close the hand
    left_gripper.set_joint_value_target(left_gripper_close)
    left_gripper.go()
    rospy.sleep(1)


def move(direction, offset):
    left_arm.shift_pose_target(direction, offset, left_eef)
    left_arm.go()
    rospy.sleep(1)


def gripper_open(status):
    if status:
        left_gripper.set_joint_value_target(left_gripper_open)
    else:
        left_gripper.set_joint_value_target(left_gripper_close)
    left_gripper.go()
    rospy.sleep(1)


def add_table(pose):
    table_id = 'table'
    # pose is in base_link frame
    # The length (0.5) and width (0.7) of table is predefined here
    size = [0.5, 0.7, (pose.pose.position.z + link_to_foot_) * 2]

    scene.remove_attached_object(reference_frame, table_id)  # Clear previous table
    scene.attach_box(reference_frame, table_id, pose, size)
    # scene.remove_world_object(table_id)
    # scene.add_box(table_id, pose, size)


def delete_table():
    table_id = 'table'
    scene.remove_attached_object(reference_frame, table_id)
    # scene.remove_world_object(table_id)  # Clear previous table


def run_grasp_ik(pose):
    # Use forward kinetic to get to initial position
    joint_pos_tgt = [0, 0, 0, 0, 1.57, 0]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(0.5)

    joint_pos_tgt = [-0.4, 0, 0, 1.57, 1.57, 0.4]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(3)

    left_gripper.set_joint_value_target(left_gripper_open)
    left_gripper.go()  # open gripper, no need for delay

    joint_pos_tgt = [0, 0, 0, 1.57, 1.57, 0]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)  # move forward
    rospy.sleep(0.5)

    # Use inverse kinetic to get to pose
    target_pose = pose  # Input pose is in base_link frame
    target_pose.header.frame_id = reference_frame
    target_pose.header.stamp = rospy.Time.now()

    left_arm.set_start_state_to_current_state()

    # Set the goal pose of the end effector to the stored pose
    left_arm.set_pose_target(target_pose, left_eef)

    # Plan the trajectory to the goal
    traj = left_arm.plan()
    n_points = len(traj.joint_trajectory.points)

    flag = Int8()
    flag.data = 0
    if n_points == 0:
        rospy.logwarn('Left arm: The traj plan failed.')
        # Back to pose [0 0 0 1.57 1.57 0]
        left_arm.set_named_target('left_arm_pose1')
        left_arm.go()
        rospy.sleep(0.5)
        # Back to initial pose [0 0 0 0 0 0]
        left_arm.set_named_target('left_arm_init')
        left_arm.go()
        flag.data = -1
    else:
        # Execute the planned trajectory
        left_arm.execute(traj)
        rospy.sleep(2)
        left_gripper.set_joint_value_target(left_gripper_close)
        left_gripper.go()
        left_arm.set_named_target('left_arm_pose1')
        left_arm.go()
        flag.data = 1
    ik_result_pub.publish(flag)


def run_grasp_fk():
    # Up to down, 6 joints, value range see neurofull_left_ikfast.urdf
    joint_pos_tgt = [0, 0, 0, 0, 1.57, 0]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(0.5)

    joint_pos_tgt = [-0.4, 0, 0, 1.57, 1.57, 0.4]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(3)

    left_gripper.set_joint_value_target(left_gripper_open)
    left_gripper.go()  # open gripper, no need for delay

    joint_pos_tgt = [0, 0, 0, 1.57, 1.57, 0]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)  # move forward
    rospy.sleep(0.5)

    joint_pos_tgt = [0.5, 0.1, 0, 0.65, 1.57, 0.4]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(2)  # move forward down

    left_gripper.set_joint_value_target(left_gripper_close)
    left_gripper.go()  # close the gripper, no delay

    joint_pos_tgt = [0.47, 0, 0, 1.2, 1.57, 0.33]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    rospy.sleep(0.5)  # move up

    joint_pos_tgt = [0, 0, 0, 1.57, 1.57, 0]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)  # let lower arm horizontal


class ArmControl:
    def __init__(self, ctrl_grasp_pose, ctrl_detect_table, ctrl_arm, feed_result,
                 use_fk):
        self._use_fk = use_fk
        self._planed = False

        # Callbacks for grasp
        self._cb_tgt = rospy.Subscriber(ctrl_grasp_pose, PoseStamped, self._target_pose_cb)
        self._cb_table = rospy.Subscriber(ctrl_detect_table, PoseStamped, self._table_cb)

        self._cb_result = rospy.Subscriber(feed_result, Int8, self._target_result_cb)

        # Callback for receiving voice command
        self._cb_reset = rospy.Subscriber(ctrl_arm, Int8, self._voice_cb)
        if use_fk:
            rospy.loginfo('Left arm: Ready to move using forward kinetic.')
        else:
            rospy.loginfo('Left arm: Ready to move using inverse kinetic.')

    def _target_pose_cb(self, pose):
        if not self._planed:
            # Recheck the method param
            if rospy.has_param('/param/arm/left/use_fk'):
                self._use_fk = rospy.get_param('/param/arm/left/use_fk')

            if self._use_fk:
                rospy.loginfo('Left arm: Using forward kinetic.')
                run_grasp_fk()
            else:
                rospy.loginfo('Left arm: Using inverse kinetic.')
                run_grasp_ik(pose)
            self._planed = True
            # Since we add table before grasp, we need remove it after grasp
            delete_table()
        else:
            pass

    @staticmethod
    def _table_cb(table):
        add_table(table)

    @staticmethod
    def _target_result_cb(data):
        # If result returns true, which means the plan has been finished, reset.
        if data.data:
            rospy.loginfo('Left arm: Finished one execution.')

    def _voice_cb(self, data):
        if data.data == 1:
            reset()
            self._planed = False
        elif data.data == 2:
            gripper_open(True)
            self._planed = False
        elif data.data == 3:
            move(0, 0.05)
        elif data.data == 4:
            move(0, -0.05)
        elif data.data == 5:
            move(1, 0.05)
        elif data.data == 6:
            move(1, -0.05)
        else:
            rospy.logwarn('Left arm: Unknown voice command.')


class NodeMain:
    def __init__(self):
        rospy.init_node('neurobot_arm_left', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Get topics names from launch file
        vision_grasp_pose = rospy.get_param('~ctrl_vision_grasp_pose', '/ctrl/vision/grasp/pose')
        vision_detect_table = rospy.get_param('~ctrl_vision_detect_table', '/ctrl/vision/detect/table')
        voice_ctrl_arm = rospy.get_param('~ctrl_voice_arm_left', '/ctrl/voice/arm/left')
        arm_feed_result = rospy.get_param('~feed_arm_grasp_result', '/feed/arm/left/move/result')
        global link_to_foot_
        link_to_foot_ = rospy.get_param('~link_to_foot', 0.465)

        # Whether use forward kinetic
        use_fk = True
        if rospy.has_param('~use_fk'):
            use_fk = rospy.get_param('~use_fk')
        else:
            rospy.logwarn("Param use_fk not available, use fk by default.")
        rospy.set_param('/param/arm/left/use_fk', use_fk)

        ArmControl(vision_grasp_pose, vision_detect_table, voice_ctrl_arm, arm_feed_result, use_fk)
        rospy.spin()

    @staticmethod
    def shutdown():
        rospy.loginfo("Left arm: Shutting down")


if __name__ == "__main__":
    try:
        NodeMain()
    except rospy.ROSInterruptException:
        rospy.loginfo("Left arm: Terminated")
