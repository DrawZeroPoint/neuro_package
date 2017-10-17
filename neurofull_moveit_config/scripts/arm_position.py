#!/usr/bin/env python
# license removed for brevity
import rospy

from moveit_msgs.msg import DisplayTrajectory

import moveit_msgs
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory,RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
#from RobotTrajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray,String,Float64

def callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #print data.MultiDOFJointTrajectoryPoint[]
    #position=RobotTrajectory
    #position=msg.trajectory
    #pub = rospy.Publisher('arm_position', moveit_msgs/RobotTrajectory[], queue_size=100)
    pub = rospy.Publisher('arm_position', Float64, queue_size=10)
    for i in range(0,6):
        pub.publish(msg.position[i])
    rospy.loginfo("i") 

def position():
    rospy.init_node('arm_position', anonymous=True)
   
    rospy.Subscriber("/joint_states", JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        position()
    except rospy.ROSInterruptException:
        pass


