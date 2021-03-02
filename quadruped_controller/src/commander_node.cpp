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

// C++
#include <map>
#include <utility>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>

// Quadruped Control
#include <quadruped_msgs/CoMState.h>
#include <quadruped_msgs/JointTorqueCmd.h>
#include <quadruped_controller/joint_controller.hpp>

using arma::vec;
using quadruped_controller::JointController;

const static std::string LOGNAME = "commander";
static bool joint_states_received = false;
static bool stand_cmd_received = false;
static vec q;
static vec v;


void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_states_received = true;
  q = msg->position;
  v = msg->velocity;
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

  ros::Publisher joint_cmd_pub =
      nh.advertise<quadruped_msgs::JointTorqueCmd>("joint_torque_cmd", 1);
  ros::Subscriber joint_sub = nh.subscribe("joint_states", 1, jointCallback);

  ros::ServiceServer start_server = nh.advertiseService("stand_up", standConfigCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Robot joint configuration
  const auto num_joints =
      static_cast<unsigned int>(pnh.param<int>("joints/num_joints", 0));
  std::vector<std::string> joint_names;
  std::vector<std::string> joint_actuator_names;
  std::vector<double> stance_joint_positions;
  std::vector<double> stance_joint_velocities;
  pnh.getParam("joints/joint_names", joint_names);
  pnh.getParam("joints/joint_actuator_names", joint_actuator_names);
  pnh.getParam("joints/stance_joint_positions", stance_joint_positions);
  pnh.getParam("joints/stance_joint_velocities", stance_joint_velocities);

  // Robot joint controller
  std::vector<double> jc_kp;
  std::vector<double> jc_kd;
  std::vector<double> tau_feed_forward;
  pnh.getParam("joint_control/kp", jc_kp);
  pnh.getParam("joint_control/kd", jc_kd);
  pnh.getParam("joint_control/tau_ff", tau_feed_forward);
  const auto tau_min = pnh.param<double>("joint_control/torque_min", -100.0);
  const auto tau_max = pnh.param<double>("joint_control/torque_max", 100.0);

  // Map joint state index to joint name/joint actuator name
  std::map<unsigned int, std::pair<std::string, std::string>> joint_map;
  // Populate joint map
  {
    unsigned int j = 0, k = 0;
    for (unsigned int i = 0; i < num_joints; i++)
    {
      if (i % 4 == 0 && i != 0)
      {
        j++;
        k = 0;
      }
      joint_map.emplace(std::make_pair(
          i, std::make_pair(joint_names.at(i), joint_actuator_names.at(j + k))));
      k += 3;
    }
  }

  // for (const auto& [key, value] : joint_map)
  // {
  //     std::cout << key << " = " << value.first << ", " << value.second << "\n";
  // }

  q.zeros(num_joints);
  v.zeros(num_joints);
  const vec qd(stance_joint_positions);
  const vec vd(stance_joint_velocities);

  // kp and kd must map to joint names
  vec kp(num_joints, arma::fill::ones);
  kp.rows(0, 3) *= jc_kp.at(0);
  kp.rows(4, 7) *= jc_kp.at(1);
  kp.rows(8, 11) *= jc_kp.at(2);
  kp.print("kp");

  vec kd(num_joints, arma::fill::ones);
  kd.rows(0, 3) *= jc_kd.at(0);
  kd.rows(4, 7) *= jc_kd.at(1);
  kd.rows(8, 11) *= jc_kd.at(2);
  kd.print("kd");

  vec tau_ff(num_joints, arma::fill::ones);
  tau_ff.rows(0, 3) *= tau_feed_forward.at(0);
  tau_ff.rows(4, 7) *= tau_feed_forward.at(1);
  tau_ff.rows(8, 11) *= tau_feed_forward.at(2);
  tau_ff.print("tau_ff");


  JointController joint_contoller(kp, kd, tau_ff, tau_min, tau_max);

  while (nh.ok())
  {
    if (stand_cmd_received)
    {
      if (joint_states_received)
      {
        const vec tau = joint_contoller.control(q, qd, v, vd);
        // tau.print("joint torque");

        quadruped_msgs::JointTorqueCmd joint_cmd;
        for (unsigned int i = 0; i < num_joints; i++)
        {
          joint_cmd.actuator_name.emplace_back(joint_map.at(i).second);
          joint_cmd.torque.emplace_back(tau(i));
        }

        joint_cmd_pub.publish(joint_cmd);

        joint_contoller.positionError().print("position error");
        // joint_contoller.velocityError().print("velocity error");
      }
      // stand_cmd_received = false;
    }
  }

  return 0;
}