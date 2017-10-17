#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "geometry_msgs/Twist.h"

//float l1=0;
//float l2=0;
//float l3=0;
//float l4=0;
//float l5=0;
//float l6=0;
//float r1=0;
//float r2=0;
//float r3=0;
//float r4=0;
//float r5=0;
//float r6=0;
std_msgs::Float32MultiArray positions;
float a[12];
void Callback1(const geometry_msgs::Twist& msg1)
{
//    l1=msg1.linear.x;
//    l2=msg1.linear.y;
//    l3=msg1.linear.z;
//    l4=msg1.angular.x;
//    l5=msg1.angular.y;
//    l6=msg1.angular.z;
       a[0]=msg1.linear.x;
       a[1]=msg1.linear.y;
        a[2]=msg1.linear.z;
        a[3]=msg1.angular.x;
        a[4]=msg1.angular.y;
         a[5]=msg1.angular.z;
//    positions.data[0] = msg1.linear.x;
//    positions.data[1] = msg1.linear.y;
//    positions.data[2] = msg1.linear.z;
//    positions.data[3] = msg1.angular.x;
//    positions.data[4] = msg1.angular.y;
//    positions.data[5] = msg1.angular.z;
}

void Callback2(const geometry_msgs::Twist& msg2)
{
//    r1=msg2.linear.x;
//    r2=msg2.linear.y;
//    r3=msg2.linear.z;
//    r4=msg2.angular.x;
//    r5=msg2.angular.y;
//    r6=msg2.angular.z;
    a[6]=msg2.linear.x;
    a[7]=msg2.linear.y;
     a[8]=msg2.linear.z;
     a[9]=msg2.angular.x;
     a[10]=msg2.angular.y;
      a[11]=msg2.angular.z;
//    positions.data[6] = msg2.linear.x;
//    positions.data[7] = msg2.linear.y;
//    positions.data[8] = msg2.linear.z;
//    positions.data[9] = msg2.angular.x;
//    positions.data[10] = msg2.angular.y;
//    positions.data[11] = msg2.angular.z;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "both_arm_pos_pub");
    ros::NodeHandle n;
    ros::Publisher both_arm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("my_trajectory_pos", 1000);
      /* ros::Subscriber<std_msgs::Float32> sub("hj_track_speedL", &sudu1_Callback);
    ros::Subscriber<std_msgs::Float32> sub1("hj_track_speedR", &sudu2_Callback);*/

    ros::Subscriber sub_left_arm = n.subscribe("left_arm_pos", 10000, Callback1);
    ros::Subscriber sub_right_arm = n.subscribe("right_arm_pos", 10000, Callback2);

    ros::Rate r(2.0);
    while(n.ok())
    {       
        ros::spinOnce();               // check for incoming messages      
        //next, we'll publish the odometry message over ROS

        //set the position
//        positions.data[0] = l1;
//        positions.data[1] = l2;
//        positions.data[2] = l3;
//        positions.data[3] = l4;
//        positions.data[4] = l5;
//        positions.data[5] = l6;
//        positions.data[6] = r1;
//        positions.data[7] = r2;
//        positions.data[8] = r3;
//        positions.data[9] = r4;
//        positions.data[10] = r5;
//        positions.data[11] = r6;
                positions.data[0] = a[0];
                positions.data[1] = a[1];
                positions.data[2] = a[2];
                positions.data[3] = a[3];
                positions.data[4] = a[4];
                positions.data[5] = a[5];
                positions.data[6] = a[6];
                positions.data[7] = a[7];
                positions.data[8] = a[8];
                positions.data[9] = a[9];
                positions.data[10] = a[10];
                positions.data[11] = a[11];
        //publish the message
        both_arm_pos_pub.publish(positions);
        r.sleep();
        }
}
