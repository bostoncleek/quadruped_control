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
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>

// Quadruped Control
#include <quadruped_controller/balance_controller.hpp>
#include <quadruped_controller/gait.hpp>
#include <quadruped_controller/foot_planner.hpp>
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

visualization_msgs::MarkerArray
footTrajViz(const FootTrajectoryManager& foot_traj_manager, const std::string& leg_name,
            double stance_phase, double t_swing)
{
  const auto steps = 30;  // steps in trajectory for visualization
  const auto dt = (1.0 - stance_phase) / steps;

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(steps);

  auto phase = stance_phase;
  for (unsigned int i = 0; i < steps; i++)
  {
    const FootState foot_state = foot_traj_manager.referenceState(leg_name, phase);

    marker_array.markers.at(i).header.frame_id = "world";
    marker_array.markers.at(i).header.stamp = ros::Time::now();
    marker_array.markers.at(i).ns = leg_name;
    marker_array.markers.at(i).id = i;
    marker_array.markers.at(i).action = visualization_msgs::Marker::ADD;
    marker_array.markers.at(i).lifetime = ros::Duration(t_swing);

    // Foot positions as points
    marker_array.markers.at(i).type = visualization_msgs::Marker::SPHERE;
    marker_array.markers.at(i).pose.position.x = foot_state.position(0);
    marker_array.markers.at(i).pose.position.y = foot_state.position(1);
    marker_array.markers.at(i).pose.position.z = foot_state.position(2);
    marker_array.markers.at(i).pose.orientation.w = 1.0;
    marker_array.markers.at(i).scale.x = 0.01;
    marker_array.markers.at(i).scale.y = 0.01;
    marker_array.markers.at(i).scale.z = 0.01;

    if (leg_name == "FL" || leg_name == "RR")
    {
      marker_array.markers.at(i).color.r = 1.0;
      marker_array.markers.at(i).color.g = 0.0;
      marker_array.markers.at(i).color.b = 0.0;
      marker_array.markers.at(i).color.a = 1.0;
    }
    else
    {
      marker_array.markers.at(i).color.r = 0.0;
      marker_array.markers.at(i).color.g = 0.0;
      marker_array.markers.at(i).color.b = 1.0;
      marker_array.markers.at(i).color.a = 1.0;
    }

    phase += dt;
  }

  return marker_array;
}

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

  ros::Publisher foot_traj_position_pub =
      nh.advertise<visualization_msgs::MarkerArray>("foot_trajectory_markers", 1);

  ros::Subscriber joint_sub = nh.subscribe("joint_states", 1, jointCallback);
  ros::Subscriber com_state_sub = nh.subscribe("com_state", 1, stateCallback);
  ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 1, cmdCallback);

  ros::ServiceServer start_server = nh.advertiseService("stand_up", standConfigCallback);

  // Broadcast post of robot in world frame
  tf2_ros::TransformBroadcaster tf_broadcaster;

  // Configure initial joint states to zeros
  for (const auto& leg_name : leg_names)
  {
    joint_states_map.emplace(leg_name, LegJointStates());
  }

  const auto frequency = pnh.param<double>("frequency", 100.0);

  // Body COM frame
  const auto base_link_name = pnh.param<std::string>("links/base_link", "trunk");

  // Gait and swing leg trajectory
  const auto t_stance = pnh.param<double>("gait/t_stance", 0.3);  // (s)
  const auto t_swing = pnh.param<double>("gait/t_swing", 0.3);    // (s)
  const auto height = pnh.param<double>("gait/height", 0.08);     // max foot height (m)
  const auto stance_phase = t_stance / (t_swing + t_stance);

  std::vector<double> gait_offset_phases = { 0.0, 0.5, 0.5, 0.0 };
  pnh.getParam("gait/gait_offset_phases", gait_offset_phases);  // [RL FL RR FR]
  const vec phase_offset(gait_offset_phases);

  // Robot joint configuration
  const auto num_joints =
      static_cast<unsigned int>(pnh.param<int>("joints/num_joints", 12));
  std::vector<std::string> joint_actuator_names;
  pnh.getParam("joints/joint_actuator_names", joint_actuator_names);

  if (num_joints != joint_actuator_names.size())
  {
    ROS_ERROR_STREAM_NAMED(
        LOGNAME, "Invalid joint configuration. Num(joints) != Num(joint_actuator_names)");

    ROS_ERROR_NAMED(LOGNAME, "Num(joints): %u, Num(joint_actuator_names names): %lu",
                    num_joints, joint_actuator_names.size());
  }

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

  std::vector<double> joint_controller_kff;
  std::vector<double> joint_controller_kp;
  std::vector<double> joint_controller_kd;
  pnh.getParam("joint_control/kff", joint_controller_kff);
  pnh.getParam("joint_control/kp", joint_controller_kp);
  pnh.getParam("joint_control/kd", joint_controller_kd);
  const vec jc_kff(joint_controller_kff);
  const vec jc_kp(joint_controller_kp);
  const vec jc_kd(joint_controller_kd);

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

  // Joint PD control for swing legs
  const JointController joint_controller(jc_kff, jc_kp, jc_kd);

  // User cmd integration step
  const auto dt = 0.001;

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

  const QuadrupedKinematics kinematics;  // Kinematic Model
  const FootPlanner foothold_planner;    // foothold planner
  const FootTrajectoryManager foot_traj_manager(height, t_swing,
                                                t_stance);  // foot trajectories
  const GaitScheduler gait_scheduler(t_swing, t_stance, phase_offset);  // gait schedule

  x_d = x_stand;
  bool standing = false;
  bool gait_running = false;

  // Use stance gait to get robot into standing configuration
  GaitMap gait_map = make_stance_gait();

  ros::Rate rate(frequency);
  while (nh.ok())
  {
    ros::spinOnce();

    // Signaled to stand
    if (stand_cmd_received)
    {
      // Robot state is known
      if (joint_states_received && com_state_received)
      {
        // FK (body frame)
        const FootholdMap foot_actual_map =
            kinematics.forwardKinematics(joint_states_map);

        // Robot is standing
        if (quadruped_controller::math::almost_equal(x(2), x_stand(2), 0.005) && !standing)
        {
          ROS_INFO_STREAM_NAMED(LOGNAME, "Standing height achieved");
          standing = true;
        }

        if (standing)
        {
          if (gait_running)
          {
            if (cmd_vel_received)
            {
              const Pose pose(Rwb, x);
              const Pose pose_desired = integrate_twist_yaw(pose, Vb, dt);

              // Desired pose
              Rwb_d = pose_desired.orientation.matrix();
              x_d = pose_desired.position;

              // TODO: height drifts
              x_d(2) = x_stand(2);

              // Desired velocities
              const vec Vw = pose.transform().adjoint() * Vb;
              xdot_d = Vw.rows(0, 2);
              w_d = Vw.rows(3, 5);

              // std::cout << "height: " << x(2) << std::endl;
              // std::cout << "desited height: " << x_d(2) << std::endl;

              // std::cout << "yaw rate actual: " << w(2) << std::endl;
              // std::cout << "yaw rate desired: " << w_d(2) << std::endl;

              // std::cout << "forward velocity: " << xdot(0) << std::endl;
              // std::cout << "forward velocity desired: " << xdot_d(0) << std::endl;

              // xdot_d.print("Desired xdot:");
              // xdot.print("Actual xdot:");

              cmd_vel_received = false;
            }

            // Foot reference states (world frame)
            FootStateMap foot_states_map;

            // Gait schedule
            gait_map = gait_scheduler.schedule();

            // Plan footholds (world frame)
            const auto foothold_plan = foothold_planner.positions(
                t_stance, Rwb, x, xdot, w, xdot_d, foot_actual_map, gait_map);

            const bool new_footholds = std::get<bool>(foothold_plan);
            const FootholdMap foothold_final_map = std::get<FootholdMap>(foothold_plan);

            if (!new_footholds)
            {
              // No planning just update reference foot states
              foot_states_map = foot_traj_manager.referenceStates(gait_map);
            }

            else
            {
              // Foot trajectory position only boundary conditions
              FootTrajBoundsMap foot_traj_bounds_map;
              for (const auto& [leg_name, p_final] : foothold_final_map)
              {
                // Transform feet from body into world frame
                const vec3 p_start = Rwb * foot_actual_map.at(leg_name) + x;
                foot_traj_bounds_map.emplace(leg_name, FootTrajBounds(p_start, p_final));
              }

              // Plan foot trajectories (world frame) and get reference states
              foot_states_map =
                  foot_traj_manager.referenceStates(gait_map, foot_traj_bounds_map);

              // Visualize foot trajectories for swing legs
              for (const auto& leg : foothold_final_map)
              {
                const visualization_msgs::MarkerArray traj_marker_msg =
                    footTrajViz(foot_traj_manager, leg.first, stance_phase, t_swing);

                foot_traj_position_pub.publish(traj_marker_msg);
              }
            }
          }

          else
          {
            gait_scheduler.start();
            gait_running = true;
          }
        }

        // Leg swing reference joint states
        JointStatesMap swing_leg_js_map;
        for (const auto& [leg_name, leg_state] : gait_map)
        {
          if (leg_state.first == LegState::swing)
          {
            FootState foot_state =
                foot_traj_manager.referenceState(leg_name, gait_map.at(leg_name).second);

            // Transform foot state into body frame for IK and J^-1
            foot_state.position = Rwb.t() * foot_state.position - x;
            foot_state.velocity = Rwb.t() * foot_state.velocity;

            const vec3 q = kinematics.legInverseKinematics(leg_name, foot_state.position);
            const vec3 qdot =
                kinematics.legJacobianInverse(leg_name, q) * foot_state.velocity;

            swing_leg_js_map.emplace(leg_name, LegJointStates(q, qdot));
          }
        }

        // Leg swing control
        const TorqueMap swing_torque_map =
            joint_controller.control(swing_leg_js_map, joint_states_map);

        // Optimize GRF for stance legs
        const ForceMap force_map = balance_controller.control(
            Rwb, Rwb_d, x, xdot, w, x_d, xdot_d, w_d, foot_actual_map, gait_map);

        // Only use for stance legs
        TorqueMap torque_map =
            kinematics.jacobianTransposeControl(joint_states_map, force_map);

        // Merge torque maps
        torque_map.insert(swing_torque_map.begin(), swing_torque_map.end());

        // control signal
        quadruped_msgs::JointTorqueCmd joint_cmd;
        for (const auto& [leg_name, torque] : torque_map)
        {
          joint_cmd.actuator_name.insert(joint_cmd.actuator_name.end(),
                                         actuator_map.at(leg_name).begin(),
                                         actuator_map.at(leg_name).end());

          // Torque limits
          const vec3 tau = arma::clamp(torque, tau_min, tau_max);
          const std::vector<double> tau_vec =
              arma::conv_to<std::vector<double>>::from(tau);

          joint_cmd.torque.insert(joint_cmd.torque.end(), tau_vec.begin(), tau_vec.end());
        }

        joint_cmd_pub.publish(joint_cmd);
      }
    }

    // Broadcast TF world to body
    if (com_state_received)
    {
      geometry_msgs::TransformStamped T_world_base;
      T_world_base.header.frame_id = "world";
      T_world_base.child_frame_id = base_link_name;
      T_world_base.header.stamp = ros::Time::now();
      T_world_base.transform.translation.x = x(0);
      T_world_base.transform.translation.y = x(1);
      T_world_base.transform.translation.z = x(2);

      const Quaternion quat_wb(Rwb);
      T_world_base.transform.rotation.x = quat_wb.x();
      T_world_base.transform.rotation.y = quat_wb.y();
      T_world_base.transform.rotation.z = quat_wb.z();
      T_world_base.transform.rotation.w = quat_wb.w();

      tf_broadcaster.sendTransform(T_world_base);
    }

    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
