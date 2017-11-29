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
  
  // The goal can be quick published, so the queue size should be reasonably bigger
  ros::Subscriber sub = n.subscribe("/left_arm_controller/follow_joint_trajectory/goal", 30, Callback);

  // Use Twist message to communication with STM32
  geometry_msgs::Twist angle_state;
  geometry_msgs::Twist vel_state;

  // Expand the queue for publishing is very important cause the traj can be transferred
  // to position and velocity command pretty quick, small queue will lead to lose intermediate data
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
      
      /**
        The velocity and angle got from traj is instantaneous in each traj point,
        including the start point and end point (velocity all equal to 0). However, the
        driver of motor needs target position and speed in range between traj points,
        so do convention here
        **/

      do {
        // Velocity for current point and the next point
        vector<double> vel_curr;
        vector<double> vel_next;
        
        // Time used for forwarding from current point to next point
        vector<ros::Duration> secs_to_next;
        
        // Angle for current point and the next point
        vector<double> angle_curr;
        vector<double> angle_next;
        
        for(int i = 0; i < act_msg->goal.trajectory.joint_names.size(); i++) {
          // For each joint, at state id = pos_count, get its state and fill it into temp
          vel_curr.push_back(act_msg->goal.trajectory.points[pos_count].velocities[i]);
          vel_next.push_back(act_msg->goal.trajectory.points[pos_count + 1].velocities[i]);
          
          secs_to_next.push_back(act_msg->goal.trajectory.points[pos_count + 1].time_from_start
                                 - act_msg->goal.trajectory.points[pos_count].time_from_start);
          
          angle_curr.push_back(act_msg->goal.trajectory.points[pos_count].positions[i]);
          angle_next.push_back(act_msg->goal.trajectory.points[pos_count + 1].positions[i]);
        }
        // Convert to Twist message, first publish velocity, then angle
        // The velocity is the instantaneous velocity at each position
        vel_state.linear.x = (angle_next[0] - angle_curr[0])/secs_to_next[0].toSec() * deceleration_;
        vel_state.linear.y = (angle_next[1] - angle_curr[1])/secs_to_next[1].toSec() * deceleration_;
        vel_state.linear.z = (angle_next[2] - angle_curr[2])/secs_to_next[2].toSec() * deceleration_;
        vel_state.angular.x = (angle_next[3] - angle_curr[3])/secs_to_next[3].toSec() * deceleration_;
        vel_state.angular.y = (angle_next[4] - angle_curr[4])/secs_to_next[4].toSec() * deceleration_;
        vel_state.angular.z = (angle_next[5] - angle_curr[5])/secs_to_next[5].toSec() * deceleration_;
        
        angle_state.linear.x = angle_next[0];
        angle_state.linear.y = angle_next[1];
        angle_state.linear.z = angle_next[2];
        angle_state.angular.x = angle_next[3];
        angle_state.angular.y = angle_next[4];
        angle_state.angular.z = angle_next[5];
        
        pub_vel_state.publish(vel_state);
        pub_angle_state.publish(angle_state);
        
        // With velocity control, there is no need to add time delay
        
        // Prepare for the next state
        pos_count++;
      }
      while(pos_count < traj_size - 1); // Omit the last point's vel

      std_msgs::Int8 flag;
      flag.data = 1;
      pub_result.publish(flag);
      ROS_INFO("Left arm: Publish trajectory plan finished.");
    }

    ros::spinOnce();
  }
  return 0;
}
