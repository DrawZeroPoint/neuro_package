#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "geometry_msgs/Twist.h"

std_msgs::Float32MultiArray positions1;
void Callback1(const geometry_msgs::Twist& msg1)
{

  positions1.data[0] = msg1.linear.x;
  positions1.data[1] = msg1.linear.y;
  positions1.data[2] = msg1.linear.z;
  positions1.data[3] = msg1.angular.x;
  positions1.data[4] = msg1.angular.y;
  positions1.data[5] = msg1.angular.z;
}

void Callback2(const geometry_msgs::Twist& msg2)
{

  positions1.data[6] = msg2.linear.x;
  positions1.data[7] = msg2.linear.y;
  positions1.data[8] = msg2.linear.z;
  positions1.data[9] = msg2.angular.x;
  positions1.data[10] = msg2.angular.y;
  positions1.data[11] = msg2.angular.z;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "both_arm_pos_pub");
  ros::NodeHandle n;
  ros::Subscriber sub_left_arm = n.subscribe("left_arm_pos", 100, Callback1);
  ros::Subscriber sub_right_arm = n.subscribe("right_arm_pos", 100, Callback2);
  ros::Publisher both_arm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("my_trajectory_pos", 1000);
  ros::Rate r(100);
  while(n.ok())
  {
    ros::spinOnce(); // check for incoming messages
    //next, we'll publish the odometry message over ROS
    //publish the message
    both_arm_pos_pub.publish(positions1);
    ros::Duration(5).sleep();
    r.sleep();
  }
}
