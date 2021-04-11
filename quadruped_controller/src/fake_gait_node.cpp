/**
 * @file gait_visualizer_node.cpp
 * @author Boston Cleek
 * @date 2021-04-7
 * @brief visualize gait
 *
 * @PARAMETERS:
 *
 * @PUBLISHES:
 * @SUBSCRIBES:
 * @SERVICES:
 */

// C++
#include <map>
#include <utility>
#include <iomanip>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>

// Quadruped Control
#include <quadruped_controller/joint_controller.hpp>
#include <quadruped_controller/balance_controller.hpp>
#include <quadruped_controller/gait.hpp>
#include <quadruped_controller/trajectory.hpp>
#include <quadruped_controller/kinematics.hpp>
#include <quadruped_controller/foot_planner.hpp>
#include <quadruped_controller/math/numerics.hpp>
#include <quadruped_msgs/CoMState.h>
#include <quadruped_msgs/JointTorqueCmd.h>

using arma::eye;
using arma::mat;
using arma::mat33;
using arma::vec;
using arma::vec3;
using std::pow;

using namespace quadruped_controller;
using namespace math;

static const std::string LOGNAME = "Gait Visualizer";

static bool cmd_vel_received = false;

// cmd body twist [vy, vy, vz, wx, wy, wz]
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gait_visualizer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Publisher foot_traj_position_pub =
      nh.advertise<visualization_msgs::MarkerArray>("foot_trajectory_markers", 1);

  ros::Publisher joint_state_pub =
      nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  ros::Subscriber cmd_sub = nh.subscribe("cmd_vel", 1, cmdCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Broadcast post of robot in world frame
  tf2_ros::TransformBroadcaster tf_broadcaster;

  // const auto frequency = pnh.param<double>("frequency", 100.0);  // (Hz)

  // Robot kinematics
  const auto base_link_name = pnh.param<std::string>("links/base_link", "trunk");

  // Legs
  std::vector<std::string> leg_names;
  pnh.getParam("legs/leg_names", leg_names);

  // Robot joint configuration
  const auto num_joints =
      static_cast<unsigned int>(pnh.param<int>("joints/num_joints", 12));
  std::vector<std::string> joint_names;
  std::vector<double> init_joint_positions;
  pnh.getParam("joints/joint_names", joint_names);
  pnh.getParam("joints/init_joint_positions", init_joint_positions);
  const vec q_init(init_joint_positions);

  if ((num_joints != joint_names.size()) || (num_joints != init_joint_positions.size()))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME,
                           "Invalid joint configuration. Num(joints) != Num(joint names) "
                           "!= Num(joint initial positions)");

    ROS_ERROR_NAMED(
        LOGNAME,
        "Num(joints): %u, Num(joint names): %lu, Num(joint initial positions): %lu",
        num_joints, joint_names.size(), init_joint_positions.size());
  }

  // Map leg name to joint names
  std::map<std::string, std::vector<std::string>> leg_joints_name_map;
  leg_joints_name_map.emplace(
      leg_names.at(0), std::vector<std::string>{ joint_names.at(0), joint_names.at(1),
                                                 joint_names.at(2) });
  leg_joints_name_map.emplace(
      leg_names.at(1), std::vector<std::string>{ joint_names.at(3), joint_names.at(4),
                                                 joint_names.at(5) });
  leg_joints_name_map.emplace(
      leg_names.at(2), std::vector<std::string>{ joint_names.at(6), joint_names.at(7),
                                                 joint_names.at(8) });
  leg_joints_name_map.emplace(
      leg_names.at(3), std::vector<std::string>{ joint_names.at(9), joint_names.at(10),
                                                 joint_names.at(11) });

  // Map leg name to initial joint positions
  std::map<std::string, std::vector<double>> leg_joints_init_positions_map;
  leg_joints_init_positions_map.emplace(
      leg_names.at(0),
      std::vector<double>{ init_joint_positions.at(0), init_joint_positions.at(1),
                           init_joint_positions.at(2) });
  leg_joints_init_positions_map.emplace(
      leg_names.at(1),
      std::vector<double>{ init_joint_positions.at(3), init_joint_positions.at(4),
                           init_joint_positions.at(5) });
  leg_joints_init_positions_map.emplace(
      leg_names.at(2),
      std::vector<double>{ init_joint_positions.at(6), init_joint_positions.at(7),
                           init_joint_positions.at(8) });
  leg_joints_init_positions_map.emplace(
      leg_names.at(3),
      std::vector<double>{ init_joint_positions.at(9), init_joint_positions.at(10),
                           init_joint_positions.at(11) });

  // Gait and swing leg trajectory
  const auto t_stance = pnh.param<double>("gait/t_stance", 1.0);  // (s)
  const auto t_swing = pnh.param<double>("gait/t_swing", 1.0);    // (s)
  const auto height = pnh.param<double>("gait/height", 0.08);     // max foot height (m)
  const auto stance_phase = t_stance / (t_swing + t_stance);

  std::vector<double> gait_offset_phases = { 0.0, 0.5, 0.5, 0.0 };
  pnh.getParam("gait/gait_offset_phases", gait_offset_phases);  // [RL FL RR FR]
  const vec phase_offset(gait_offset_phases);

  // Robot state in world frame
  std::vector<double> position = { 0.0, 0.0, 0.0 };
  std::vector<double> orientation = { 0.0, 0.0, 0.0, 1.0 };
  std::vector<double> linear_velocity = { 0.0, 0.0, 0.0, 0.0 };
  pnh.getParam("robot_state/position", position);
  pnh.getParam("robot_state/orientation", orientation);
  pnh.getParam("robot_state/linear_velocity", linear_velocity);

  mat33 Rwb = Quaternion(orientation.at(3), orientation.at(0), orientation.at(1),
                         orientation.at(2))
                  .matrix();
  vec x(position);
  vec xdot(linear_velocity);

  geometry_msgs::TransformStamped T_world_base;
  T_world_base.header.frame_id = "world";
  T_world_base.child_frame_id = base_link_name;

  // TODO: fix const in QuadrupedKinematics
  QuadrupedKinematics kinematics;  // FK and IK

  // Foot start positions in world frame
  mat ft_p_init = kinematics.forwardKinematics(q_init);
  ft_p_init.col(0) = Rwb * ft_p_init.col(0) + x;
  ft_p_init.col(1) = Rwb * ft_p_init.col(1) + x;
  ft_p_init.col(2) = Rwb * ft_p_init.col(2) + x;
  ft_p_init.col(3) = Rwb * ft_p_init.col(3) + x;

  FootholdMap foothold_start_map;
  foothold_start_map.emplace("RL", ft_p_init.col(0));
  foothold_start_map.emplace("FL", ft_p_init.col(1));
  foothold_start_map.emplace("RR", ft_p_init.col(2));
  foothold_start_map.emplace("FR", ft_p_init.col(3));
  // ft_p_init.print("ft_p_init");

  const FootPlanner foothold_planner;  // foothold planner
  const FootTrajectoryManager foot_traj_manager(height, t_swing,
                                                t_stance);  // foot trajectories

  const GaitScheduler gait_scheduler(t_swing, t_stance, phase_offset);  // gait schedule
  gait_scheduler.start();

  const auto dt = 0.05;  // integrate twist (s)

  // ros::Rate rate(frequency);
  while (nh.ok())
  {
    if (cmd_vel_received)
    {
      const Pose pose(Rwb, x);
      const Pose pose_desired = integrate_twist_yaw(pose, Vb, dt);

      // Desired pose
      Rwb = pose_desired.orientation.matrix();
      x = pose_desired.position;

      cmd_vel_received = false;
    }

    // Publish joint states
    for (const auto& leg_name : leg_names)
    {
      sensor_msgs::JointState joint_states_msg;

      joint_states_msg.header.stamp = ros::Time::now();
      joint_states_msg.name = leg_joints_name_map.at(leg_name);
      joint_states_msg.position = leg_joints_init_positions_map.at(leg_name);
      joint_state_pub.publish(joint_states_msg);
    }

    // Broadcast robot pose
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

    // rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
