#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <math.h>

Servo left_hand_servo;

float got = 0.1; // In radius
float servo_position = 10.0;

ros::NodeHandle nh;

void poseCallback(const std_msgs::Float32& msg)
{
  got = msg.data;
}

ros::Subscriber<std_msgs::Float32> sub_l("gripper_pos_left", &poseCallback);

void setup()
{
  Serial.begin(57600);
  delay(10);
  left_hand_servo.attach(9);
  left_hand_servo.write(servo_position);
  ros_int();//初始化ros
}

void loop()
{
  servo_position = got * 57.3;
  nh.spinOnce();
  run_servo();
  delay(10);
}

void ros_int()
{
  nh.initNode();
  nh.subscribe(sub_l);
}

void run_servo()
{
  left_hand_servo.write(servo_position);
}
