/**
 * @file commander_node.cpp
 * @author Boston Cleek
 * @date 2021-02-21
 * @brief Main Quadruped command scheduler
 *
 * @PARAMETERS:
 *
 * @PUBLISHES:
 *    joint_torque_cmd (quadruped_msgs/JointTorqueCmd) - joint torques
 * @SUBSCRIBES:
 *    joint_states (sensor_msgs/JointState) - joint names, positions, and velocities
 *    com_state (quadruped_msgs/CoMState) - COM pose and velocity twist in world frame
 *    cmd_vel (geometry_msgs/Twist) - user commanded body twist
 * @SERVICES:
 *    stand_up (std_srvs/Empty) - triggers robot to stand up
 */

// C++
#include <map>
#include <utility>
#include <iomanip>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

// Quadruped Control
#include <quadruped_controller/balance_controller.hpp>
#include <quadruped_controller/gait.hpp>
#include <quadruped_controller/joint_controller.hpp>
#include <quadruped_controller/kinematics.hpp>
#include <quadruped_controller/trajectory.hpp>
#include <quadruped_controller/math/numerics.hpp>
#include <quadruped_msgs/CoMState.h>
#include <quadruped_msgs/JointTorqueCmd.h>

using arma::eye;
using arma::mat;
using arma::vec;
using arma::vec3;

using namespace quadruped_controller;
using namespace math;

const static std::string LOGNAME = "commander";

static bool joint_states_received = false;
static bool com_state_received = false;
static bool stand_cmd_received = false;
static bool cmd_vel_received = false;

// IMPORTANT: Most of the software has been configured to run
//            with these joint names and in this order
static std::vector<std::string> leg_names = { "RL", "FL", "RR", "FR" };

// Actual State
static JointStatesMap joint_states_map;  // q and qdot

static mat Rwb = eye(3, 3);           // COM orientation
static vec3 x(arma::fill::zeros);     // COM position
static vec3 xdot(arma::fill::zeros);  // COM linear velocity
static vec3 w(arma::fill::zeros);     // COM angular velocity

// Cmd
// body twist [vy, vy, vz, wx, wy, wz]
static vec Vb(6, arma::fill::zeros);

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_states_received = true;

  // RL
  joint_states_map.at("RL").q(0) = msg->position.at(0);  // RL_hip_joint
  joint_states_map.at("RL").q(1) = msg->position.at(4);  // RL_thigh_joint
  joint_states_map.at("RL").q(2) = msg->position.at(8);  // RL_calf_joint

  joint_states_map.at("RL").qdot(0) = msg->velocity.at(0);  // FL_hip_joint
  joint_states_map.at("RL").qdot(1) = msg->velocity.at(4);  // FL_thigh_joint
  joint_states_map.at("RL").qdot(2) = msg->velocity.at(8);  // FL_calf_joint

  // FL
  joint_states_map.at("FL").q(0) = msg->position.at(1);
  joint_states_map.at("FL").q(1) = msg->position.at(5);
  joint_states_map.at("FL").q(2) = msg->position.at(9);

  joint_states_map.at("FL").qdot(0) = msg->velocity.at(1);
  joint_states_map.at("FL").qdot(1) = msg->velocity.at(5);
  joint_states_map.at("FL").qdot(2) = msg->velocity.at(9);

  // RR
  joint_states_map.at("RR").q(0) = msg->position.at(2);
  joint_states_map.at("RR").q(1) = msg->position.at(6);
  joint_states_map.at("RR").q(2) = msg->position.at(10);

  joint_states_map.at("RR").qdot(0) = msg->velocity.at(2);
  joint_states_map.at("RR").qdot(1) = msg->velocity.at(6);
  joint_states_map.at("RR").qdot(2) = msg->velocity.at(10);

  // FR
  joint_states_map.at("FR").q(0) = msg->position.at(3);
  joint_states_map.at("FR").q(1) = msg->position.at(7);
  joint_states_map.at("FR").q(2) = msg->position.at(11);

  joint_states_map.at("FR").qdot(0) = msg->velocity.at(3);
  joint_states_map.at("FR").qdot(1) = msg->velocity.at(7);
  joint_states_map.at("FR").qdot(2) = msg->velocity.at(11);
}

void stateCallback(const quadruped_msgs::CoMState::ConstPtr& msg)
{
  com_state_received = true;

  Quaternion quat(msg->pose.orientation.w, msg->pose.orientation.x,
                  msg->pose.orientation.y, msg->pose.orientation.z);

  Rwb = quat.rotation().matrix();

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

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_received = true;

  Vb(0) = msg->linear.x;
  Vb(1) = msg->linear.y;
  Vb(2) = msg->linear.z;

  Vb(3) = msg->angular.x;
  Vb(4) = msg->angular.y;
  Vb(5) = msg->angular.z;
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
  ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 1, cmdCallback);

  ros::ServiceServer start_server = nh.advertiseService("stand_up", standConfigCallback);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Configure initial joint states to zeros
  for (const auto& leg_name : leg_names)
  {
    joint_states_map.emplace(leg_name, LegJointStates());
  }

  // Robot joint configuration
  const auto num_joints =
      static_cast<unsigned int>(pnh.param<int>("joints/num_joints", 12));
  std::vector<std::string> joint_actuator_names;
  pnh.getParam("joints/joint_actuator_names", joint_actuator_names);

  // map leg name to actuator names
  std::map<std::string, std::vector<std::string>> actuator_map;
  actuator_map.emplace(leg_names.at(0),
                       std::vector<std::string>{ joint_actuator_names.at(0),
                                                 joint_actuator_names.at(1),
                                                 joint_actuator_names.at(2) });
  actuator_map.emplace(leg_names.at(1),
                       std::vector<std::string>{ joint_actuator_names.at(3),
                                                 joint_actuator_names.at(4),
                                                 joint_actuator_names.at(5) });
  actuator_map.emplace(leg_names.at(2),
                       std::vector<std::string>{ joint_actuator_names.at(6),
                                                 joint_actuator_names.at(7),
                                                 joint_actuator_names.at(8) });
  actuator_map.emplace(leg_names.at(3),
                       std::vector<std::string>{ joint_actuator_names.at(9),
                                                 joint_actuator_names.at(10),
                                                 joint_actuator_names.at(11) });

  // Robot controller
  double w_diagonal = 1e-5;
  std::vector<double> s_diagonal;
  std::vector<double> kff_gains;
  std::vector<double> kp_com_p;
  std::vector<double> kp_com_w;
  std::vector<double> kd_com_p;
  std::vector<double> kd_com_w;
  pnh.getParam("balance_control/w_diagonal", w_diagonal);
  pnh.getParam("balance_control/s_diagonal", s_diagonal);
  pnh.getParam("balance_control/kff", kff_gains);
  pnh.getParam("balance_control/kp_p", kp_com_p);
  pnh.getParam("balance_control/kp_w", kp_com_w);
  pnh.getParam("balance_control/kd_p", kd_com_p);
  pnh.getParam("balance_control/kd_w", kd_com_w);

  // Weight on forces in f.T*W*f
  const mat W = eye(12, 12) * w_diagonal;
  // Weight on least squares (Ax-b)*S*(Ax-b)
  const mat S = arma::diagmat(vec(s_diagonal));
  const vec kff(kff_gains);  // feed forward gains
  const vec kp_p(kp_com_p);  // COM position Kp
  const vec kp_w(kp_com_w);  // COM orientation Kp
  const vec kd_p(kd_com_p);  // COM linear velocity Kd
  const vec kd_w(kd_com_w);  // COM angular velocity Kd

  // std::vector<double> jc_kp;
  // std::vector<double> jc_kd;
  // std::vector<double> tau_feed_forward;
  // pnh.getParam("joint_control/kp", jc_kp);
  // pnh.getParam("joint_control/kd", jc_kd);
  // pnh.getParam("joint_control/tau_ff", tau_feed_forward);
  const auto frequency = pnh.param<double>("balance_control/frequency", 100.0);
  const auto tau_min = pnh.param<double>("balance_control/torque_min", -20.0);
  const auto tau_max = pnh.param<double>("balance_control/torque_max", 20.0);

  // Dynamic properties
  std::vector<double> inertia_body;
  pnh.getParam("dynamics/Ib", inertia_body);
  const mat Ib = arma::diagmat(vec(inertia_body));
  const auto mu = pnh.param<double>("dynamics/mu", 0.8);
  const auto mass = pnh.param<double>("dynamics/mass", 11.0);
  const auto fzmin = pnh.param<double>("dynamics/fzmin", 10.0);
  const auto fzmax = pnh.param<double>("dynamics/fzmax", 160.0);

  // GRF Control
  const BalanceController balance_controller(mu, mass, fzmin, fzmax, Ib, S, W, kff, kp_p,
                                             kd_p, kp_w, kd_w, leg_names);

  // Kinematic Model
  const QuadrupedKinematics kinematics;

  // User cmd integration step
  // const auto dt = 0.05;

  // Hard code the desired COM state to standing configuration
  mat Rwb_d = eye(3, 3);           // base orientation in world
  vec3 x_d(arma::fill::zeros);     // position in world
  vec3 xdot_d(arma::fill::zeros);  // linear velocity
  vec3 w_d(arma::fill::zeros);     // angular velocity

  // Default standing state
  const mat Rwb_stand = eye(3, 3);
  const vec3 x_stand = { 0., 0., 0.26 };
  const vec3 xdot_stand(arma::fill::zeros);
  const vec3 w_stand(arma::fill::zeros);

  x_d = x_stand;
  bool standing = false;

  // stance gait used to get robot into standing configuration
  GaitMap gait_map = make_stance_gait();

  ros::Rate rate(frequency);
  while (nh.ok())
  {
    // Signaled to stand
    if (stand_cmd_received)
    {
      // Robot state is known
      if (joint_states_received && com_state_received)
      {
        // Robot is standing
        if (quadruped_controller::math::almost_equal(x(2), x_stand(2), 0.005) && !standing)
        {
          ROS_INFO_STREAM_NAMED(LOGNAME, "Standing height achieved");
          standing = true;
        }

        // FK in body frame
        const FootholdMap foot_FK = kinematics.forwardKinematics(joint_states_map);

        // Optimize GRF for stance legs
        const ForceMap force_map = balance_controller.control(
            Rwb, Rwb_d, x, xdot, w, x_d, xdot_d, w_d, foot_FK, gait_map);

        // Only use for stance legs
        TorqueMap torque_map =
            kinematics.jacobianTransposeControl(joint_states_map, force_map);

        // Send control signal
        quadruped_msgs::JointTorqueCmd joint_cmd;
        // joint_cmd.actuator_name.resize(num_joints);
        // joint_cmd.torque.resize(num_joints);

        for (const auto& [leg_name, torque] : torque_map)
        {
          joint_cmd.actuator_name.insert(joint_cmd.actuator_name.end(),
                                         actuator_map.at(leg_name).begin(),
                                         actuator_map.at(leg_name).end());

          // Torque limits
          const vec3 tau = arma::clamp(torque, tau_min, tau_max);
          const std::vector<double> tau_vec =
              arma::conv_to<std::vector<double>>::from(tau);
          // tau.print("tau");

          joint_cmd.torque.insert(joint_cmd.torque.end(), tau_vec.begin(), tau_vec.end());
        }

        joint_cmd_pub.publish(joint_cmd);
      }
    }

    rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
