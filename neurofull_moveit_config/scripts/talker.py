#!/usr/bin/env python
# license removed for brevity
import rospy
#import Pose
from geometry_msgs.msg import Pose,Point,Quaternion

def talker():
    pub = rospy.Publisher('chatter', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pose=Pose(Point(0.3,-0.5,0.88),Quaternion(0.56,0.33,-0.29,-0.15))
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

