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
#include "brics_actuator/JointVelocities.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

static const string JOINTNAME_PRE = "left_arm_joint";
static const uint NUM_ARM_JOINTS = 6;
vector<control_msgs::FollowJointTrajectoryActionGoal::ConstPtr> trajectories;

void Callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  ROS_INFO("callback: Trajectory received");
  trajectories.push_back(msg);
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"right_arm_velocities_controller");
  ros::NodeHandle n;
  uint loop_counter = 0;
  brics_actuator::JointVelocities command;
  std_msgs::Float32MultiArray  traj;
  geometry_msgs::Twist traj1_vel;
  vector <brics_actuator::JointValue> armJointVelocities;
  armJointVelocities.resize(NUM_ARM_JOINTS);
  command.velocities = armJointVelocities;
  ros::Subscriber my_sub=n.subscribe("/left_arm_controller/follow_joint_trajectory/goal",1000,Callback);
  ros::Publisher armVelocitiesPublisher=n.advertise<brics_actuator::JointVelocities>("right_arm_joints",10);
  ros::Publisher my_pub1=n.advertise<geometry_msgs::Twist>("my_trajectory_left_vel",100);
  ros::Rate r(10);
  ros::Duration(1).sleep();
  ros::spinOnce();

  while (n.ok())
  {
    if(!trajectories.empty())
    {
      control_msgs::FollowJointTrajectoryActionGoal::ConstPtr act_msg;
      ROS_INFO("new Trajectory");
      act_msg = trajectories.front();//返回当前vector容器中起始元素的引用
      trajectories.erase(trajectories.begin());//从指定容器删除指定位置的元素或某段范围内的元素
      //cout << "Msg-Header" << endl << *act_msg << endl;

      armJointVelocities.resize(act_msg->goal.trajectory.joint_names.size());
      for(int i = 0; i<act_msg->goal.trajectory.joint_names.size();i++)
      {
        armJointVelocities[i].joint_uri = act_msg->goal.trajectory.joint_names[i];
        armJointVelocities[i].unit = boost::units::to_string(boost::units::si::radians);
      }
      command.velocities = armJointVelocities;

      uint pos_count = 0;

      /// prepare first point
      for(int i = 0; i<act_msg->goal.trajectory.joint_names.size();i++)
      {
        command.velocities[i].value = act_msg->goal.trajectory.points[pos_count].velocities[i];
        //traj.data[i]=(float)act_msg->goal.trajectory.points[pos_count].Velocities[i];
      }
      traj1_vel.linear.x=command.velocities[0].value;
      traj1_vel.linear.y=command.velocities[1].value;
      traj1_vel.linear.z=command.velocities[2].value;
      traj1_vel.angular.x=command.velocities[3].value;
      traj1_vel.angular.y=command.velocities[4].value;
      traj1_vel.angular.z=command.velocities[5].value;
      ros::Time start_time = ros::Time::now();

      do
      {
        // send point
        armVelocitiesPublisher.publish(command);
        my_pub1.publish(traj1_vel);
        // prepare next point
        pos_count++;
        for(int i = 0; i<act_msg->goal.trajectory.joint_names.size();i++)
        {
          command.velocities[i].value = act_msg->goal.trajectory.points[pos_count].velocities[i];
          //traj.data[i]=(float)act_msg->goal.trajectory.points[pos_count].Velocities[i];
        }
        traj1_vel.linear.x=command.velocities[0].value;
        traj1_vel.linear.y=command.velocities[1].value;
        traj1_vel.linear.z=command.velocities[2].value;
        traj1_vel.angular.x=command.velocities[3].value;
        traj1_vel.angular.y=command.velocities[4].value;
        traj1_vel.angular.z=command.velocities[5].value;
        //sleep
        ros::Duration((start_time+act_msg->goal.trajectory.points[pos_count].time_from_start)
                      -ros::Time::now()).sleep();

      }
      while(pos_count < act_msg->goal.trajectory.points.size()-1);
      traj1_vel.linear.x=command.velocities[0].value;
      traj1_vel.linear.y=command.velocities[1].value;
      traj1_vel.linear.z=command.velocities[2].value;
      traj1_vel.angular.x=command.velocities[3].value;
      traj1_vel.angular.y=command.velocities[4].value;
      traj1_vel.angular.z=command.velocities[5].value;
      armVelocitiesPublisher.publish(command);
      my_pub1.publish(traj1_vel);
      ros::Duration(0.5).sleep();

    }
    ros::Duration(1).sleep();
    ros::spinOnce();
    //ROS_INFO("Loop count %d", loop_counter);
    //armVelocitiesPublisher.publish(command);
    loop_counter++;
  }

  return 0;

}
