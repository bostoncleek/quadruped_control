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
#include <quadruped_controller/joint_controller.hpp>
#include <quadruped_controller/balance_controller.hpp>
#include <quadruped_controller/kinematics.hpp>
#include <quadruped_msgs/CoMState.h>
#include <quadruped_msgs/JointTorqueCmd.h>

using arma::eye;
using arma::mat;
using arma::vec;

using quadruped_controller::BalanceController;
using quadruped_controller::JointController;
using quadruped_controller::QuadrupedKinematics;
using quadruped_controller::rigid3d::Quaternion;
using quadruped_controller::rigid3d::Rotation3d;

const static std::string LOGNAME = "commander";

static bool joint_states_received = false;
static bool com_state_received = false;
static bool stand_cmd_received = false;

static vec q(12, arma::fill::zeros);  // joint angles
static vec v(12, arma::fill::zeros);  // joint velocities

static mat Rwb = eye(3, 3);             // COM orientation
static vec x(3, arma::fill::zeros);     // COM position
static vec xdot(3, arma::fill::zeros);  // COM linear velocity
static vec w(3, arma::fill::zeros);     // COM angular velocity


void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_states_received = true;

  // RL
  q(0) = msg->position.at(0);  // RL_hip_joint
  q(1) = msg->position.at(4);  // RL_thigh_joint
  q(2) = msg->position.at(8);  // RL_calf_joint

  v(0) = msg->velocity.at(0);
  v(1) = msg->velocity.at(4);
  v(2) = msg->velocity.at(8);

  // FL
  q(3) = msg->position.at(1);  // FL_hip_joint
  q(4) = msg->position.at(5);  // FL_thigh_joint
  q(5) = msg->position.at(9);  // FL_calf_joint

  v(3) = msg->velocity.at(1);
  v(4) = msg->velocity.at(5);
  v(5) = msg->velocity.at(9);

  // RR
  q(6) = msg->position.at(2);   // RR_hip_joint
  q(7) = msg->position.at(6);   // RR_thigh_joint
  q(8) = msg->position.at(10);  // RR_calf_joint

  v(6) = msg->velocity.at(2);
  v(7) = msg->velocity.at(6);
  v(8) = msg->velocity.at(10);

  // FR
  q(9) = msg->position.at(3);    // FR_hip_joint
  q(10) = msg->position.at(7);   // FR_thigh_joint
  q(11) = msg->position.at(11);  // FR_calf_joint

  v(9) = msg->velocity.at(3);
  v(10) = msg->velocity.at(7);
  v(11) = msg->velocity.at(11);
}


void stateCallback(const quadruped_msgs::CoMState::ConstPtr& msg)
{
  com_state_received = true;

  Quaternion quat(msg->pose.orientation.w, msg->pose.orientation.x,
                  msg->pose.orientation.y, msg->pose.orientation.z);

  Rwb = quat.rotation().rotationMatrix();

  x(0) = msg->pose.position.x;
  x(1) = msg->pose.position.y;
  x(2) = msg->pose.position.z;

  xdot(0) = msg->twist.linear.x;
  xdot(1) = msg->twist.linear.y;
  xdot(2) = msg->twist.linear.z;

  w(0) = msg->twist.angular.x;
  w(1) = msg->twist.angular.y;
  w(2) = msg->twist.angular.z;
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
  ros::Subscriber com_state_sub = nh.subscribe("com_state", 1, stateCallback);

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
  // std::vector<double> jc_kp;
  // std::vector<double> jc_kd;
  // std::vector<double> tau_feed_forward;
  // pnh.getParam("joint_control/kp", jc_kp);
  // pnh.getParam("joint_control/kd", jc_kd);
  // pnh.getParam("joint_control/tau_ff", tau_feed_forward);
  const auto tau_min = pnh.param<double>("joint_control/torque_min", -100.0);
  const auto tau_max = pnh.param<double>("joint_control/torque_max", 100.0);

  // Dynamic properties
  const auto mu = 0.8;    // Coefficient of friction
  const auto mass = 9.0;  // total mass

  const auto fzmin = 10.0;   // absolute min z-direction reaction force
  const auto fzmax = 160.0;  // absolute max z-direction reaction force

  mat Ib(3, 3, arma::fill::zeros);  // inertia of base
  Ib(0, 0) = 0.011253;              // Ixx
  Ib(1, 1) = 0.036203;              // Iyy
  Ib(2, 2) = 0.042673;              // Izz

  // Control Gains and QP Weights
  mat S = eye(6, 6);  // Weight on least squares (Ax-b)*S*(Ax-b)
  // S(0, 0) = 100;
  // S(1, 1) = 100;
  // S(2, 2) = 100;

  // S(3, 3) = 100;
  // S(4, 4) = 100;
  // S(5, 5) = 100;

  const vec kp_p = { 200., 200., 200. };     // COM position Kp
  const vec kp_w = { 2000., 2000., 2000. };  // COM orientation Kp

  const vec kd_p = { 50., 50., 50. };     // COM linear velocity Kd
  const vec kd_w = { 200., 200., 200. };  // COM angular velocity Kd

  // GRF Control
  BalanceController balance_controller(mu, mass, fzmin, fzmax, Ib, S, kp_p, kd_p, kp_w,
                                       kd_w);

  // Kinematic Model
  QuadrupedKinematics kinematics;

  // Hard code the desired COM state to standing configuration
  Quaternion qwb_d;                               //(0.966, 0., 0., 0.259);
  mat Rwb_d = qwb_d.rotation().rotationMatrix();  // base orientation in world

  vec x_d = { 0., 0., 0.35 };        // position in world
  vec xdot_d(3, arma::fill::zeros);  // linear velocity
  vec w_d(3, arma::fill::zeros);     // angular velocity

  while (nh.ok())
  {
    if (stand_cmd_received)
    {
      if (joint_states_received && com_state_received)
      {
        const mat ft_p = kinematics.forwardKinematics(q);
        // ft_p.print("ft_p");

        const vec fb =
            balance_controller.control(ft_p, Rwb, Rwb_d, x, xdot, w, x_d, xdot_d, w_d);
        // fb.print("fb");

        const vec tau = kinematics.jacobianTransposeControl(q, fb);
        // tau.print("tau");

        quadruped_msgs::JointTorqueCmd joint_cmd;
        joint_cmd.actuator_name = joint_actuator_names;
        joint_cmd.torque = arma::conv_to<std::vector<double>>::from(tau);

        joint_cmd_pub.publish(joint_cmd);
      }
      // stand_cmd_received = false;
    }
  }

  return 0;
}
