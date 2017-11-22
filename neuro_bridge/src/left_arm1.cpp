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
#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

static const string JOINTNAME_PRE = "left_joint_";
static const uint NUM_ARM_JOINTS = 6;
vector<control_msgs::FollowJointTrajectoryActionGoal::ConstPtr> trajectories;

void Callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  ROS_INFO("callback: Trajectory received");
  //cout << "Msg-Header" << endl << msg->header << endl;
  trajectories.push_back(msg);
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"trackarm_trajectory_controller");
  ros::NodeHandle n;
  uint loop_counter = 0;
  int k=10;
  brics_actuator::JointPositions command;
  std_msgs::Float64MultiArray  traj;
  float between[6];
  float each[6];
  geometry_msgs::Twist traj1;
  vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(NUM_ARM_JOINTS);
  command.positions = armJointPositions;
  ros::Subscriber my_sub=n.subscribe("/left_arm_controller/follow_joint_trajectory/goal",1000,Callback);
  ros::Publisher armPositionsPublisher=n.advertise<brics_actuator::JointPositions>("left_arm_joints",10);
  //ros::Publisher my_pub=n.advertise<std_msgs::Float64MultiArray>("arm_pos_left",10);
  ros::Publisher my_pub1=n.advertise<geometry_msgs::Twist>("my_trajectory_pos_left",100);
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

      armJointPositions.resize(act_msg->goal.trajectory.joint_names.size());
      for(int i = 0; i<act_msg->goal.trajectory.joint_names.size();i++)
      {
        armJointPositions[i].joint_uri = act_msg->goal.trajectory.joint_names[i];
        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
      }
      command.positions = armJointPositions;

      for(int pos_count = 0;pos_count < act_msg->goal.trajectory.points.size();pos_count++)
      {

        for(int i=0; i<act_msg->goal.trajectory.joint_names.size();i++)
        {
          command.positions[i].value = act_msg->goal.trajectory.points[pos_count].positions[i];
          between[i]=act_msg->goal.trajectory.points[pos_count+1].positions[i]-act_msg->goal.trajectory.points[pos_count].positions[i];
          each[i]=between[i]/k;
        }
        for(int j=0;j<k;j++)
        {
          traj1.linear.x=command.positions[0].value + each[0]*j;
          traj1.linear.y=command.positions[1].value + each[1]*j;
          traj1.linear.z=command.positions[2].value + each[2]*j;
          traj1.angular.x=command.positions[3].value+ each[3]*j;
          traj1.angular.y=command.positions[4].value + each[4]*j;
          traj1.angular.z=command.positions[5].value + each[5]*j;
          my_pub1.publish(traj1);
        }
        //ros::Duration((start_time+act_msg->goal.trajectory.points[pos_count].time_from_start)-ros::Time::now()).sleep();
        ros::Duration(0.5).sleep();
      }
      for(int i=0; i<act_msg->goal.trajectory.joint_names.size();i++)
      {
        command.positions[i].value = act_msg->goal.trajectory.points[act_msg->goal.trajectory.points.size()].positions[i];
      }
      traj1.linear.x=command.positions[0].value;
      traj1.linear.y=command.positions[1].value;
      traj1.linear.z=command.positions[2].value;
      traj1.angular.x=command.positions[3].value;
      traj1.angular.y=command.positions[4].value;
      traj1.angular.z=command.positions[5].value;
      my_pub1.publish(traj1);

    }
    ros::Duration(1).sleep();
    ros::spinOnce();
    //ROS_INFO("Loop count %d", loop_counter);
    //armPositionsPublisher.publish(command);
    loop_counter++;
  }

  return 0;

}
