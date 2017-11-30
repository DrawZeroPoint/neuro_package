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

// When the traj callback is receiving traj,
// the transfer will not be executed
bool on_receiving_ = false;

// Record the last traj received time
double lastTrajTime_ = 0.0;

// Reduce the plan speed by this coeff
double deceleration_ = 1.0;

double delayTime_ = 1.0;

void Callback(const TrajPtr& msg)
{
  trajectories.push_back(msg);
  on_receiving_ = true;
}

bool receiveTimeOut()
{
  if (on_receiving_) {
    lastTrajTime_ = ros::Time::now().toSec();
    return false; // Means have not been timeout
  }
  else {
    double currTime = ros::Time::now().toSec();
    if (currTime - lastTrajTime_ > delayTime_) {
      return true;
    }
    else
      return false;
  }
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"left_arm_trajectory_controller");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  
  pnh.getParam("deceleration", deceleration_);
  pnh.getParam("delay", delayTime_);
  
  // The goal can be quick published, so the queue size should be reasonably bigger
  ros::Subscriber sub = n.subscribe("/left_arm_controller/follow_joint_trajectory/goal", 100, Callback);
  
  // Use Twist message to communication with STM32
  geometry_msgs::Twist angle_state;
  geometry_msgs::Twist vel_state;
  
  // Expand the queue for publishing is very important cause the traj can be transferred
  // to position and velocity command pretty quick, small queue will lead to lose intermediate data
  ros::Publisher pub_angle_state = n.advertise<geometry_msgs::Twist>("my_trajectory_pos_left", 100);
  ros::Publisher pub_vel_state = n.advertise<geometry_msgs::Twist>("my_trajectory_vel_left", 100);
  
  // Result feedback
  ros::Publisher pub_result = n.advertise<std_msgs::Int8>("/feed/arm/left/move/result", 1);
  
  while (n.ok()) {
    if(!trajectories.empty() && !on_receiving_) {
      ROS_INFO("Left arm: Get new trajectories from planning.");
      
      for (size_t t = 0; t < trajectories.size(); ++t) {
        TrajPtr act_msg;
        
        // The act_msg contains 1 trajectory from planning
        act_msg = trajectories[t];
        
        uint pos_count = 0; // Count state number
        int traj_size = act_msg->goal.trajectory.points.size();
        
        /**
          The velocity and angle got from traj is instantaneous in each traj point,
          including the start point and end point (velocity all equal to 0). However, the
          driver of motor needs target position and speed in range between traj points,
          besides, ##!!!! THE DRIVER ONLY RECOGNIZE POSITIVE SPEED , BUT THE SPEED 
          FROM PLANNING CAN BE POSITIVE OR NEGATIVE, SO DO CONVENTION HERE !!!!##
          **/
        
        do {
          // Velocity for current point and the next point
          vector<double> vel_curr;
          vector<double> vel_next;
          
          // Time used for forwarding from current point to next point
          ros::Duration secs_to_next;
          
          // Angle for current point and the next point
          vector<double> angle_curr;
          vector<double> angle_next;
          
          for(int i = 0; i < act_msg->goal.trajectory.joint_names.size(); i++) {
            // For each joint, at state id = pos_count, get its state and fill it into temp
            vel_curr.push_back(act_msg->goal.trajectory.points[pos_count].velocities[i]);
            vel_next.push_back(act_msg->goal.trajectory.points[pos_count + 1].velocities[i]);
            
            angle_curr.push_back(act_msg->goal.trajectory.points[pos_count].positions[i]);
            
            //angle_next.push_back(act_msg->goal.trajectory.points[pos_count + 1].positions[i]);
            // Test: always use the last pose
            angle_next.push_back(act_msg->goal.trajectory.points[traj_size - 1].positions[i]);
          }
          secs_to_next = act_msg->goal.trajectory.points[pos_count + 1].time_from_start
              - act_msg->goal.trajectory.points[pos_count].time_from_start;
          // Convert to Twist message, first publish velocity, then angle
          // The velocity is the instantaneous velocity at each position
          
          // Method 1: use average speed calculated by: d/t
          //vel_state.linear.x = (angle_next[0] - angle_curr[0])/secs_to_next.toSec() * deceleration_;
          //vel_state.linear.y = (angle_next[1] - angle_curr[1])/secs_to_next.toSec() * deceleration_;
          //vel_state.linear.z = (angle_next[2] - angle_curr[2])/secs_to_next.toSec() * deceleration_;
          //vel_state.angular.x = (angle_next[3] - angle_curr[3])/secs_to_next.toSec() * deceleration_;
          //vel_state.angular.y = (angle_next[4] - angle_curr[4])/secs_to_next.toSec() * deceleration_;
          //vel_state.angular.z = (angle_next[5] - angle_curr[5])/secs_to_next.toSec() * deceleration_;
          
          // Method 2: always use the next speed
          vel_state.linear.x = fabs(vel_next[0]);
          vel_state.linear.y = fabs(vel_next[1]);
          vel_state.linear.z = fabs(vel_next[2]);
          vel_state.angular.x = fabs(vel_next[3]);
          vel_state.angular.y = fabs(vel_next[4]);
          vel_state.angular.z = fabs(vel_next[5]);
          
          angle_state.linear.x = angle_next[0];
          angle_state.linear.y = angle_next[1];
          angle_state.linear.z = angle_next[2];
          angle_state.angular.x = angle_next[3];
          angle_state.angular.y = angle_next[4];
          angle_state.angular.z = angle_next[5];
          
          pub_vel_state.publish(vel_state);
          pub_angle_state.publish(angle_state);
          
          // No nedd to add time delay
          ros::Duration d = act_msg->goal.trajectory.points[traj_size - 1].time_from_start
                            - act_msg->goal.trajectory.points[0].time_from_start;
          d.sleep();
          // Prepare for the next state
          pos_count++;
        }
        while(pos_count < traj_size - 1); // Omit the last point's velocity cause it is 0
      }
      // All traj have been published, so we need clear the temp
      trajectories.clear();
      
      // Publish feedback
      std_msgs::Int8 flag;
      flag.data = 1;
      pub_result.publish(flag);
      ROS_INFO("Left arm: Publish trajectory plan finished.");
    }
    
    // Reset the receive status before each loop
    on_receiving_ = false;
    ros::spinOnce();
    
    // Judge whether the traj have been completely received
    if (receiveTimeOut)
      on_receiving_ = false;
    else
      on_receiving_ = true;
  }
  return 0;
}
