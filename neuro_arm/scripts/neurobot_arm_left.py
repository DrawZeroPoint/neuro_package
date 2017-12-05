#!/usr/bin/env python

import rospy
import sys
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_commander
from moveit_commander import PlanningSceneInterface
# from moveit_msgs.msg import CollisionObject, Grasp

from std_msgs.msg import Int8, UInt16
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped

left_gripper_open = [1.5]
left_gripper_close = [0.3]

# Recommended table height: 0.89
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
left_arm.set_goal_position_tolerance(0.012)
left_arm.set_goal_orientation_tolerance(0.012)

# Construct the initial scene object
scene = PlanningSceneInterface()
map_frame = 'map'  # Reference frame for scene

# Publish inverse kinetic result
ik_result_pub = rospy.Publisher('/feed/arm/left/plan/result', Int8, queue_size=1)
# Publish signal when start moving
signal_pub = rospy.Publisher('/vision/error', UInt16, queue_size=1)
# Param to control is putting down object
param_is_put = '/comm/param/ctrl/is_put'

# Current status
current_status = 'left_arm_init'

# Try pick place function of moveit
# Publish transform for debug
# rosrun tf static_transform_publisher --args=0 0 0 0 0 0 odom base_footprint 100


# Get the coord near the target which has same orientation with the target
def get_prepare_pose(offset, target_pose):
    # Get Euler angle from orientation
    orientation_q = target_pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw = 1
    # For now we assume only yaw!=0
    pose = PoseStamped()
    pose.header = target_pose.header
    pose.pose.position.x = target_pose.pose.position.x - offset * np.cos(yaw)
    pose.pose.position.y = target_pose.pose.position.y - offset * np.sin(yaw)
    pose.pose.position.z = target_pose.pose.position.z
    pose.pose.orientation = target_pose.pose.orientation
    return pose


def pub_signal(code):
    # Publish start signal
    signal = UInt16()
    signal.data = code
    signal_pub.publish()


def add_table(pose):
    table_id = 'table'
    # pose is in base_link frame
    # The length (0.5) and width (0.7) of table is predefined here
    size = [0.5, 0.7, (pose.pose.position.z + link_to_foot_) * 2]

    # We can not use attach method cause the pose frame we want to use (odom) is not in robot frame list
    scene.remove_world_object(table_id)
    scene.add_box(table_id, pose, size)


def delete_table():
    table_id = 'table'
    scene.remove_world_object(table_id)  # Clear previous table


def move(direction, offset):
    left_arm.shift_pose_target(direction, offset, left_eef)
    left_arm.go()


def gripper_open(status):
    if status:
        left_gripper.set_joint_value_target(left_gripper_open)
    else:
        left_gripper.set_joint_value_target(left_gripper_close)
    left_gripper.go()


def reset():
    pub_signal(20)  # Orange flash for starting
    left_arm.clear_pose_targets()
    # Reset using inverse kinetic
    left_arm.set_named_target('left_arm_init')
    left_arm.go()
    global current_status
    current_status = 'left_arm_init'
    # Close the hand
    gripper_open(False)


def to_prepare_pose():
    left_arm.set_named_target('left_arm_pose_pre')
    left_arm.go()
    global current_status
    current_status = 'left_arm_pose_pre'


def ik_result_check_and_run(traj):
    # Check whether ik returns useful result,
    # if true, execute it, else back to initial pose
    traj_num = len(traj.joint_trajectory.points)

    flag = Int8()
    flag.data = 0
    if traj_num == 0:
        rospy.logwarn('Left arm: The traj plan failed.')
        flag.data = -1
    else:
        # Execute the planned trajectory
        left_arm.execute(traj)
        flag.data = 1
    ik_result_pub.publish(flag)
    return flag.data


def run_grasp_ik(pose):
    if current_status != 'left_arm_pose_pre':
        # Use forward kinetic to get to initial position
        # Open gripper, no need for delay
        gripper_open(True)
        to_prepare_pose()

    # First get near to the target
    target_pose_pre = get_prepare_pose(0.1, pose)

    # Set the goal pose of the end effector to the prepare pose
    left_arm.set_pose_target(target_pose_pre, left_eef)

    # Plan the trajectory to the goal
    traj = left_arm.plan()
    if ik_result_check_and_run(traj):
        # Set the goal pose of the end effector to the stored pose
        left_arm.set_pose_target(pose, left_eef)

        # Plan the trajectory to the goal
        traj = left_arm.plan()

        if ik_result_check_and_run(traj):
            # Wait to be steady
            rospy.sleep(1)
            gripper_open(False)
            # move backward
            left_arm.set_named_target('left_arm_pose1')
            left_arm.go()
        else:
            rospy.logwarn('Left arm: No plan for final pose.')
    else:
        rospy.logwarn('Left arm: No plan for prepare pose.')

    # All situation end up in pose 1
    to_prepare_pose()


def run_put_ik(pose):
    if current_status != 'left_arm_pose_pre':
        # Not in prepare pose means no object to put
        rospy.set_param(param_is_put, 0)
        rospy.logwarn('Left arm: Put will not be executed due to inappropriate pose.')
        return
    # First get near to the put pose
    put_pose_pre = pose  # Input pose is in base_link frame
    put_pose_pre.header.frame_id = reference_frame
    put_pose_pre.header.stamp = rospy.Time.now()
    # Notice that in python, -= will also influence pose
    put_pose_pre.pose.position.z += 0.02

    # Set the goal pose of the end effector to the prepare pose
    left_arm.set_pose_target(put_pose_pre, left_eef)

    # Plan the trajectory to the goal
    traj = left_arm.plan()
    if ik_result_check_and_run(traj):
        # Input pose is in base_link frame
        put_pose = pose
        put_pose.header.frame_id = reference_frame
        put_pose.header.stamp = rospy.Time.now()
        put_pose.pose.position.z -= 0.02

        # Set the goal pose of the end effector to the stored pose
        left_arm.set_pose_target(put_pose, left_eef)

        # Plan the trajectory to the goal
        traj = left_arm.plan()
        if ik_result_check_and_run(traj):
            # Open gripper and drop the object
            gripper_open(True)
            # Wait to be steady
            rospy.sleep(1)
            # Move backward
            move(0, -0.1)
        else:
            rospy.logwarn('Left arm: No plan for final put.')
    else:
        rospy.logwarn('Left arm: No plan for prepare putting.')

    # Back to prepare pose
    to_prepare_pose()


def run_put_fk(pose):
    if current_status != 'left_arm_pose_pre':
        # Not in prepare pose means no object to put
        rospy.set_param(param_is_put, 0)
        rospy.logwarn('Left arm: Put will not be executed due to inappropriate pose.')
        return
    # move forward down
    joint_pos_tgt = [0, 0, 0, 1.64, 1.57, 0.4]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    joint_pos_tgt = [0.5, 0, 0, 0.65, 1.57, 0.4]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    # Wait to be steady
    rospy.sleep(1)
    # Open gripper and drop the object
    gripper_open(True)

    # Back to prepare pose
    to_prepare_pose()


def run_grasp_fk():
    if current_status != 'left_arm_pose_pre':
        # Use forward kinetic to get to initial position
        # Open gripper, no need for delay
        gripper_open(True)
        to_prepare_pose()

    # move forward down
    joint_pos_tgt = [0.5, 0.1, 0, 0.65, 1.57, 0.4]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)

    gripper_open(False)  # close the gripper, no delay

    # move up
    joint_pos_tgt = [0.47, 0, 0, 1.2, 1.57, 0.33]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    # Back to prepare pose
    to_prepare_pose()


def run_test(joint_id):
    # Test each joint to see if it has lost power
    joint_pos_tgt = [0, 0, 0, 0, 0, 0]
    if joint_id - 90 == 1:
        joint_pos_tgt = [0.1, 0, 0, 0, 0, 0]
    if joint_id - 90 == 2:
        joint_pos_tgt = [0, 0.1, 0, 0, 0, 0]
    if joint_id - 90 == 3:
        joint_pos_tgt = [0, 0, 0.1, 0, 0, 0]
    if joint_id - 90 == 4:
        joint_pos_tgt = [0, 0, 0, 0.1, 0, 0]
    if joint_id - 90 == 5:
        joint_pos_tgt = [0, 0, 0, 0, 0.1, 0]
    if joint_id - 90 == 6:
        joint_pos_tgt = [0, 0, 0, 0, 0, 0.1]
    left_arm.set_joint_value_target(joint_pos_tgt)
    traj = left_arm.plan()
    left_arm.execute(traj)
    global current_status
    current_status = 'left_arm_init'


class ArmControl:
    def __init__(self, ctrl_grasp_pose, ctrl_grasp_location, ctrl_detect_table, ctrl_put_pose,
                 ctrl_arm, feed_result, use_fk):
        self._use_fk = use_fk
        self._planed = False

        # Callbacks for grasp
        self._cb_tgt = rospy.Subscriber(ctrl_grasp_pose, PoseStamped, self._target_pose_cb, queue_size=1)
        self._cb_tgt_loc = rospy.Subscriber(ctrl_grasp_location, PoseStamped, self._target_loc_cb, queue_size=1)

        self._cb_table = rospy.Subscriber(ctrl_detect_table, PoseStamped, self._table_cb, queue_size=1)
        self._cb_put = rospy.Subscriber(ctrl_put_pose, PoseStamped, self._put_pose_cb, queue_size=1)

        self._cb_result = rospy.Subscriber(feed_result, Int8, self._target_result_cb)

        # Callback for receiving voice command
        self._cb_reset = rospy.Subscriber(ctrl_arm, Int8, self._voice_cb)

        # Service that clear the octomap
        rospy.loginfo('Waiting for clear_octomap')
        rospy.wait_for_service('/clear_octomap', 5)
        self._clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        rospy.loginfo('Got clear_octomap')

        if use_fk:
            rospy.loginfo('Left arm: Ready to move using forward kinetic.')
        else:
            rospy.loginfo('Left arm: Ready to move using inverse kinetic.')

    def _target_pose_cb(self, pose):
        if not self._planed:
            # Recheck the method param
            if rospy.has_param('/arm/param/left/use_fk'):
                self._use_fk = rospy.get_param('/arm/param/left/use_fk')

            pub_signal(1)  # Yellow for starting
            # Clear octomap generated during approaching to the target before grasping
            self._clear_octomap()
            if self._use_fk:
                rospy.loginfo('Left arm: Using forward kinetic.')
                run_grasp_fk()
            else:
                rospy.loginfo('Left arm: Using inverse kinetic.')
                run_grasp_ik(pose)
            self._planed = True
            # Since we add table before grasp, we need remove it after grasp
            delete_table()
            pub_signal(0)  # Green for finish
        else:
            pass

    def _target_loc_cb(self, pose):
        # self._clear_octomap()
        pass

    def _put_pose_cb(self, pose):
        if self._planed:
            # _planed not being reset means we didn't release the object in hand
            # so that we can put it down, meanwhile, we can reset _planed and the arm
            self._planed = False
            pub_signal(1)  # Yellow for starting
            self._clear_octomap()
            run_put_fk(pose)
            # run_put_ik(pose)
            pub_signal(0)  # Green for finish

    @staticmethod
    def _table_cb(table):
        add_table(table)

    @staticmethod
    def _target_result_cb(data):
        # If result returns true, which means the plan has been finished, reset.
        if data.data:
            rospy.loginfo('Left arm: Finished plan execution.')

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
        elif 96 >= data.data >= 91:
            run_test(data.data)
        else:
            rospy.logwarn('Left arm: Unknown voice command.')


class NodeMain:
    def __init__(self):
        rospy.init_node('neurobot_arm_left', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Get topics names from launch file
        vision_grasp_pose = rospy.get_param('~ctrl_vision_grasp_pose', '/ctrl/vision/grasp/pose')
        vision_grasp_loc = rospy.get_param('~ctrl_vision_grasp_location', '/ctrl/vision/grasp/location')

        vision_detect_table = rospy.get_param('~ctrl_vision_detect_table', '/ctrl/vision/detect/table')
        vision_put_pose = rospy.get_param('~ctrl_vision_put_pose', '/ctrl/vision/put/pose')
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
        rospy.set_param('/arm/param/left/use_fk', use_fk)

        ArmControl(vision_grasp_pose, vision_grasp_loc, vision_detect_table, vision_put_pose,
                   voice_ctrl_arm, arm_feed_result, use_fk)
        rospy.spin()

    @staticmethod
    def shutdown():
        rospy.loginfo("Left arm: Shutting down")


if __name__ == "__main__":
    try:
        NodeMain()
    except rospy.ROSInterruptException:
        rospy.loginfo("Left arm: Terminated")
