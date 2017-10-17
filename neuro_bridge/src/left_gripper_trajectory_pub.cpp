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

static const double INIT_POS[] = {0,0};
static const string JOINTNAME_PRE = "left_joint_gripper_";
static const uint NUM_gripper_JOINTS = 1;
vector<control_msgs::FollowJointTrajectoryActionGoal::ConstPtr> trajectories_gripper;

void Callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  ROS_INFO("callback: Trajectory received");
  //cout << "Msg-Header" << endl << msg->header << endl;
  trajectories_gripper.push_back(msg);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"trackgripper_trajectory_controller");
    ros::NodeHandle n;
    uint loop_counter = 0;
    brics_actuator::JointPositions command_gripper;
    std_msgs::Float32 traj_gripper;
    vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(NUM_gripper_JOINTS);
    command_gripper.positions = gripperJointPositions;
    ros::Subscriber gripper_pos_sub=n.subscribe("/left_gripper_controller/follow_joint_trajectory/goal",1000,Callback);
    ros::Publisher gripperPositionsPublisher=n.advertise<brics_actuator::JointPositions>("gripper_joints",10);
    //ros::Publisher my_pub=n.advertise<std_msgs::Float32MultiArray>("gripper_pos",10);
    ros::Publisher gripper_pub=n.advertise<std_msgs::Float32>("gripper_pos",10);
    ros::Rate r(100);
    ros::Duration(1).sleep();
    ros::spinOnce();

      while (n.ok())
      {
        if(!trajectories_gripper.empty())
        {
          control_msgs::FollowJointTrajectoryActionGoal::ConstPtr act_msg_gripper;
          ROS_INFO("new Trajectory");
          act_msg_gripper = trajectories_gripper.front();//返回当前vector容器中起始元素的引用
          trajectories_gripper.erase(trajectories_gripper.begin());//从指定容器删除指定位置的元素或某段范围内的元素
          //cout << "Msg-Header" << endl << *act_msg << endl;

          gripperJointPositions.resize(act_msg_gripper->goal.trajectory.joint_names.size());
          for(int i = 0; i<act_msg_gripper->goal.trajectory.joint_names.size();i++)
          {
            gripperJointPositions[i].joint_uri = act_msg_gripper->goal.trajectory.joint_names[i];
            gripperJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
          }
          command_gripper.positions = gripperJointPositions;

          uint pos_count_gripper = 0;

          /// prepare first point
          for(int i = 0; i<act_msg_gripper->goal.trajectory.joint_names.size();i++)
          {
            command_gripper.positions[i].value = act_msg_gripper->goal.trajectory.points[pos_count_gripper].positions[i];
            //traj.data[i]=(float)act_msg->goal.trajectory.points[pos_count].positions[i];
          }
          traj_gripper.data=command_gripper.positions[0].value;
          
          ros::Time start_time = ros::Time::now();

          do
          {
            // send point
            gripperPositionsPublisher.publish(command_gripper);
            gripper_pub.publish(traj_gripper);
            // prepare next point
            pos_count_gripper++;
            for(int i = 0; i<act_msg_gripper->goal.trajectory.joint_names.size();i++)
            {
              command_gripper.positions[i].value = act_msg_gripper->goal.trajectory.points[pos_count_gripper].positions[i];
              //traj.data[i]=(float)act_msg->goal.trajectory.points[pos_count].positions[i];
            }
            traj_gripper.data=command_gripper.positions[0].value;
           
            //sleep
            ros::Duration((start_time+act_msg_gripper->goal.trajectory.points[pos_count_gripper].time_from_start)-ros::Time::now()).sleep();

          }
          while(pos_count_gripper < act_msg_gripper->goal.trajectory.points.size()-1);
         traj_gripper.data=command_gripper.positions[0].value;
         
          gripperPositionsPublisher.publish(command_gripper);
          gripper_pub.publish(traj_gripper);
          ros::Duration(0.05).sleep();

        }
        ros::Duration(1).sleep();
        ros::spinOnce();
        //ROS_INFO("Loop count %d", loop_counter);
        //gripperPositionsPublisher.publish(command);
        loop_counter++;
        }

        return 0;

}
