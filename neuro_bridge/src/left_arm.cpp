#include "ros/ros.h"
#include <std_msgs/Int8.h>
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

typedef control_msgs::FollowJointTrajectoryActionGoal::ConstPtr TrajPtr;

vector<TrajPtr> trajectories;

// Reduce the plan speed by this coeff
double deceleration_ = 1.0;
void Callback(const TrajPtr& msg)
{
  ROS_INFO("Left arm: Trajectory plan received.");
  trajectories.push_back(msg);
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"left_arm_trajectory_controller");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  
  pnh.getParam("deceleration", deceleration_);
  
  ros::Subscriber sub = n.subscribe("/left_arm_controller/follow_joint_trajectory/goal", 1, Callback);

  // Use Twist message to communication with STM32
  geometry_msgs::Twist angle_state;
  geometry_msgs::Twist vel_state;

  // Expand the queue
  ros::Publisher pub_angle_state = n.advertise<geometry_msgs::Twist>("my_trajectory_pos_left", 30);
  ros::Publisher pub_vel_state = n.advertise<geometry_msgs::Twist>("my_trajectory_vel_left", 30);

  // Result feedback
  ros::Publisher pub_result = n.advertise<std_msgs::Int8>("/feed/arm/left/move/result", 1);

  while (n.ok()) {
    if(!trajectories.empty()) {
      TrajPtr act_msg;
      ROS_INFO("Left arm: Get new trajectory from planning.");
      
      // The act_msg contains all trajectories from planning
      act_msg = trajectories.front();
      // Clear the temp for the next planning
      trajectories.erase(trajectories.begin());

      uint pos_count = 0; // Count state number
      int traj_size = act_msg->goal.trajectory.points.size();

      do {
        vector<float> state_vel_temp;
        vector<float> state_angle_temp;
        
        for(int i = 0; i < act_msg->goal.trajectory.joint_names.size(); i++) {
          // For each joint, at state id = pos_count, get its state and fill it into temp
          state_vel_temp.push_back(act_msg->goal.trajectory.points[pos_count].velocities[i]);
          state_angle_temp.push_back(act_msg->goal.trajectory.points[pos_count].positions[i]);
        }
        // Convert to Twist message, first publish velocity, then angle
        // The velocity is the instantaneous velocity at each position
        vel_state.linear.x = state_vel_temp[0] * deceleration_;
        vel_state.linear.y = state_vel_temp[1] * deceleration_;
        vel_state.linear.z = state_vel_temp[2] * deceleration_;
        vel_state.angular.x = state_vel_temp[3] * deceleration_;
        vel_state.angular.y = state_vel_temp[4] * deceleration_;
        vel_state.angular.z = state_vel_temp[5] * deceleration_;
        
        angle_state.linear.x = state_angle_temp[0];
        angle_state.linear.y = state_angle_temp[1];
        angle_state.linear.z = state_angle_temp[2];
        angle_state.angular.x = state_angle_temp[3];
        angle_state.angular.y = state_angle_temp[4];
        angle_state.angular.z = state_angle_temp[5];
        
//        pub_vel_state.publish(vel_state);
        pub_angle_state.publish(angle_state);
        
        // With velocity control, there is no need to add time delay
        
        // Prepare for the next state
        pos_count++;
      }
      while(pos_count < traj_size);

      std_msgs::Int8 flag;
      flag.data = 1;
      pub_result.publish(flag);
      ROS_INFO("Left arm: Publish trajectory plan finished.");
    }

    ros::spinOnce();
  }
  return 0;
}
