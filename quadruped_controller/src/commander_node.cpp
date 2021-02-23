/**
 * @file commander_node.cpp
 * @author Boston Cleek
 * @date 2021-02-21
 * @brief Main Quadruped command scheduler 
 *
 * @PARAMETERS:
 *
 * @PUBLISHES:
 *
 * @SUBSCRIBES:
 *
 */

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

// Quadruped Control
#include <quadruped_msgs/CoMState.h>
#include <quadruped_msgs/JointTorqueCmd.h>
#include <quadruped_controller/pid_controller.hpp>

const static std::string LOGNAME = "commander";
static bool stand_cmd_received = false;

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

}


bool standConfigCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Commading robot to standing configuration");
  stand_cmd_received = true; 
  return true;
}


int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting commander node");
  ros::init(argc, argv, "commander");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Publisher joint_cmd_pub = nh.advertise<quadruped_msgs::JointTorqueCmd>("joint_torque_cmd", 1);
  ros::Subscriber joint_sub = nh.subscribe("joint_states", 1, jointCallback);

  ros::ServiceServer start_server = nh.advertiseService("stand_up", standConfigCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Robot joint configuration
  const auto num_joints =
      static_cast<unsigned int>(pnh.param<int>("joints/num_joints", 0));
  std::vector<std::string> joint_actuator_names;
  std::vector<double> stance_joint_positions;
  std::vector<double> stance_joint_velocities;
  pnh.getParam("joints/joint_actuator_names", joint_actuator_names);
  pnh.getParam("joints/stance_joint_positions", stance_joint_positions);
  pnh.getParam("joints/stance_joint_velocities", stance_joint_velocities);


  while (nh.ok())
  {
    if (stand_cmd_received)
    {

      stand_cmd_received = false;
    }
  }

  return 0;
}