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
#include <quadruped_controller/joint_controller.hpp>
#include <quadruped_controller/balance_controller.hpp>
#include <quadruped_controller/trajectory.hpp>
#include <quadruped_controller/kinematics.hpp>
#include <quadruped_controller/math/numerics.hpp>
#include <quadruped_msgs/CoMState.h>
#include <quadruped_msgs/JointTorqueCmd.h>

using arma::eye;
using arma::mat;
using arma::vec;
using arma::vec3;

using quadruped_controller::BalanceController;
using quadruped_controller::JointController;
using quadruped_controller::QuadrupedKinematics;
using quadruped_controller::math::Pose;
using quadruped_controller::math::Quaternion;
using quadruped_controller::math::Rotation3d;

const static std::string LOGNAME = "commander";

static bool joint_states_received = false;
static bool com_state_received = false;
static bool stand_cmd_received = false;
static bool cmd_vel_received = false;

// Actual State
static vec q(12, arma::fill::zeros);     // joint angles
static vec qdot(12, arma::fill::zeros);  // joint velocities

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
  q(0) = msg->position.at(0);  // RL_hip_joint
  q(1) = msg->position.at(4);  // RL_thigh_joint
  q(2) = msg->position.at(8);  // RL_calf_joint

  qdot(0) = msg->velocity.at(0);
  qdot(1) = msg->velocity.at(4);
  qdot(2) = msg->velocity.at(8);

  // FL
  q(3) = msg->position.at(1);  // FL_hip_joint
  q(4) = msg->position.at(5);  // FL_thigh_joint
  q(5) = msg->position.at(9);  // FL_calf_joint

  qdot(3) = msg->velocity.at(1);
  qdot(4) = msg->velocity.at(5);
  qdot(5) = msg->velocity.at(9);

  // RR
  q(6) = msg->position.at(2);   // RR_hip_joint
  q(7) = msg->position.at(6);   // RR_thigh_joint
  q(8) = msg->position.at(10);  // RR_calf_joint

  qdot(6) = msg->velocity.at(2);
  qdot(7) = msg->velocity.at(6);
  qdot(8) = msg->velocity.at(10);

  // FR
  q(9) = msg->position.at(3);    // FR_hip_joint
  q(10) = msg->position.at(7);   // FR_thigh_joint
  q(11) = msg->position.at(11);  // FR_calf_joint

  qdot(9) = msg->velocity.at(3);
  qdot(10) = msg->velocity.at(7);
  qdot(11) = msg->velocity.at(11);
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

  // Robot joint configuration
  // const auto num_joints =
  //     static_cast<unsigned int>(pnh.param<int>("joints/num_joints", 0));
  std::vector<std::string> joint_actuator_names;
  pnh.getParam("joints/joint_actuator_names", joint_actuator_names);

  // Robot controller
  double w_diagonal = 1e-5;
  std::vector<double> s_diagonal;
  std::vector<double> kff_gains;
  std::vector<double> kp_com_p;
  std::vector<double> kp_com_w;
  std::vector<double> kd_com_p;
  std::vector<double> kd_com_w;
  pnh.getParam("control/w_diagonal", w_diagonal);
  pnh.getParam("control/s_diagonal", s_diagonal);
  pnh.getParam("control/kff", kff_gains);
  pnh.getParam("control/kp_p", kp_com_p);
  pnh.getParam("control/kp_w", kp_com_w);
  pnh.getParam("control/kd_p", kd_com_p);
  pnh.getParam("control/kd_w", kd_com_w);

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
  const auto frequency = pnh.param<double>("control/frequency", 100.0);
  const auto tau_min = pnh.param<double>("control/torque_min", -20.0);
  const auto tau_max = pnh.param<double>("control/torque_max", 20.0);

  // Dynamic properties
  const auto mu = 1.0;  // Coefficient of friction

  // Mass in URDF is 11kg
  const auto mass = 9.0;  // total mass

  const auto fzmin = 10.0;   // absolute min z-direction reaction force
  const auto fzmax = 160.0;  // absolute max z-direction reaction force

  mat Ib(3, 3, arma::fill::zeros);  // inertia of base
  Ib(0, 0) = 0.011253;              // Ixx
  Ib(1, 1) = 0.036203;              // Iyy
  Ib(2, 2) = 0.042673;              // Izz

  // GRF Control
  const BalanceController balance_controller(mu, mass, fzmin, fzmax, Ib, S, W, kff, kp_p,
                                             kd_p, kp_w, kd_w);

  // Kinematic Model
  QuadrupedKinematics kinematics;

  // Stance base control (only when standing)
  quadruped_controller::StanceBaseControl stance_control;

  // User cmd integration step
  const auto dt = 0.05;

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

  ros::Rate rate(frequency);
  while (nh.ok())
  {
    if (stand_cmd_received)
    {
      if (joint_states_received && com_state_received)
      {
        if (quadruped_controller::math::almost_equal(x(2), x_stand(2), 0.005) && !standing)
        {
          const Pose pose(Rwb, x);
          stance_control.setPose(pose);

          ROS_INFO_STREAM_NAMED(LOGNAME, "Standing height achieved");
          standing = true;
        }

        // Set desired states base on user cmd
        if (cmd_vel_received && standing)
        {
          // Walking configuration
          // const vec cmd = {Vb(0), Vb(1), Vb(2), 0.0, 0.0, Vb(5)};

          // Stand configuration
          const vec cmd = { 0.0, 0.0, Vb(2), Vb(3), Vb(4), Vb(5) };

          const Pose pose(Rwb, x);
          // const Pose pose_desired =
          //         quadruped_controller::integrate_twist_yaw(pose, cmd, dt);

          // Do not update the pose very iteration to prevent drift
          const Pose pose_desired = stance_control.integrateTwist(pose, cmd, dt);

          // Desired pose
          Rwb_d = pose_desired.orientation.matrix();
          x_d = pose_desired.position;

          // Desired velocities
          const vec Vw = pose.transform().adjoint() * cmd;
          xdot_d = Vw.rows(0, 2);
          w_d = Vw.rows(3, 5);

          // std::cout << "Trunk height: " << x(2) << std::endl;
          // std::cout << "Desired Trunk height: " << x_d(2) << std::endl;

          // (pose.orientation.eulerAngles() * 180.0 / M_PI).print("actual orientation
          // (deg)"); pose.position.print("actual position (m)");

          // (pose_desired.orientation.eulerAngles() * 180.0 / M_PI).print("orientation
          // desired (deg)"); pose_desired.position.print("position desired (m)");

          cmd_vel_received = false;
        }

        const mat ft_p = kinematics.forwardKinematics(q);
        // ft_p.print("ft_p");

        const vec fb =
            balance_controller.control(ft_p, Rwb, Rwb_d, x, xdot, w, x_d, xdot_d, w_d);
        // fb.print("fb");

        vec tau = kinematics.jacobianTransposeControl(q, fb);

        // Torque limits
        tau = arma::clamp(tau, tau_min, tau_max);
        // tau.print("tau");

        quadruped_msgs::JointTorqueCmd joint_cmd;
        joint_cmd.actuator_name = joint_actuator_names;
        joint_cmd.torque = arma::conv_to<std::vector<double>>::from(tau);

        joint_cmd_pub.publish(joint_cmd);
      }
    }

    rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
