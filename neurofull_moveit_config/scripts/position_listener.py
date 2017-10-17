import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
def callback(data):
    global a,b,c,d,e,f,a[100]
    a[]=data.position
    a=a[0]
    b=a[1]
    c=a[2]
    d=a[3]
    e=a[4]
    f=a[5]
    print a,b,c,d,e,f
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('position_listener', anonymous=True)

    rospy.Subscriber("/joint_states", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
