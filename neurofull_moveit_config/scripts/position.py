#!/usr/bin/env python
# license removed for brevity
import rospy
# import Pose
# from geometry_msgs.msg import Pose,Point,Quaternion
from moveit_msgs.msg import DisplayTrajectory

import moveit_msgs
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectory
# from RobotTrajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray, String


def callback(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print data.MultiDOFJointTrajectoryPoint[]
    # position=RobotTrajectory
    # position=msg.trajectory
    # pub = rospy.Publisher('arm_position', moveit_msgs/RobotTrajectory[], queue_size=100)
    pub = rospy.Publisher('arm_position', RobotTrajectory, queue_size=10)
    pub.publish(msg.trajectory[0])
    # n= len (data.joint_trajectory.points)
    # print msg.trajectory
    # print msg.joint_trajectory.points[1].positions
    # print msg.trajectory_start.joint_state.position
    # print data.multi_dof_joint_trajectory.points
    rospy.loginfo("i")


def position():
    rospy.init_node('position', anonymous=True)

    rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, callback)
    # position=DisplayTrajectory
    # position.trajectory=msg.trajectory
    # pub = rospy.Publisher('arm_position', moveit_msgs/RobotTrajectory[], queue_size=100)
    # pub = rospy.Publisher('arm_position', RobotTrajectory, queue_size=10)
    # position=DisplayTrajectory
    # pub.publish(position.trajectory)
    rospy.spin()


if __name__ == '__main__':
    try:
        position()
    except rospy.ROSInterruptException:
        pass
