#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <math.h>

Servo left_hand_servo;

float got = 0.1; // In radius
float servo_position = 10.0;
bool new_msg_received = false;

ros::NodeHandle nh;

void poseCallback(const std_msgs::Float32& msg)
{
  got = msg.data; // 手的规划
  new_msg_received = true;
}

ros::Subscriber<std_msgs::Float32> sub_l("gripper_pos_left", &poseCallback);

void setup()
{
  Serial.begin(57600); // arduino与上位机的通信波特率
  delay(10);
  left_hand_servo.attach(9);
  left_hand_servo.write(servo_position);
  ros_int();//初始化ros

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  servo_position = got * 57.3;
  nh.spinOnce();
  if (new_msg_received)
    run_servo();
  new_msg_received = false;
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
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1500);                       // wait for 1.5 second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1500);
}
