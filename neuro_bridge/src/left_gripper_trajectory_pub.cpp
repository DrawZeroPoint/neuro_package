#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include "moveit_msgs/RobotState.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include <moveit_msgs/PlanningScene.h>

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

vector<control_msgs::FollowJointTrajectoryActionGoal::ConstPtr> trajectories_gripper;

void Callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  ROS_INFO("Left gripper: Trajectory plan received.");
  trajectories_gripper.push_back(msg);
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"trackgripper_trajectory_controller");
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("/left_gripper_controller/follow_joint_trajectory/goal", 1, Callback);
  
  std_msgs::Float32 gripper_state;
  ros::Publisher gripper_pub = n.advertise<std_msgs::Float32>("gripper_pos", 1);
  
  while (n.ok()) {
    if(!trajectories_gripper.empty()) {
      control_msgs::FollowJointTrajectoryActionGoal::ConstPtr act_msg_gripper;
      ROS_INFO("Left gripper: Get new trajectory from planning.");
      
      act_msg_gripper = trajectories_gripper.front();
      trajectories_gripper.erase(trajectories_gripper.begin());
      
      uint pos_count_gripper = 0;
      
      do {
        vector<float> state_gripper_temp;
        for(int i = 0; i < act_msg_gripper->goal.trajectory.joint_names.size();i++) {
          state_gripper_temp.push_back(act_msg_gripper->goal.trajectory.points[pos_count_gripper].positions[i]);
        }
        // Send state to gripper
        gripper_state.data = state_gripper_temp[0];
        gripper_pub.publish(gripper_state);
        
        pos_count_gripper++;
      }
      while(pos_count_gripper < act_msg_gripper->goal.trajectory.points.size());
      ROS_INFO("Left gripper: Publish trajectory plan finished.");
    }
    ros::spinOnce();
  }
  return 0;
}
