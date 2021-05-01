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
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>

// Quadruped Control
#include <quadruped_controller/balance_controller.hpp>
#include <quadruped_controller/gait.hpp>
#include <quadruped_controller/foot_planner.hpp>
#include <quadruped_controller/kinematics.hpp>
#include <quadruped_controller/trajectory.hpp>
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

static const std::string LOGNAME = "gait_visualizer";

visualization_msgs::MarkerArray
footTrajViz(const FootTrajectoryManager& foot_traj_manager, const std::string& leg_name,
            double stance_phase, double t_swing)
{
  const auto steps = 30;  // steps in trajectory for visualization
  const auto dt = (1.0 - stance_phase) / steps;

  visualization_msgs::MarkerArray position_marker_array;
  position_marker_array.markers.resize(steps);

  auto phase = stance_phase;
  for (unsigned int i = 0; i < steps; i++)
  {
    const FootState foot_state = foot_traj_manager.referenceState(leg_name, phase);
    position_marker_array.markers.at(i).header.frame_id = "world";
    position_marker_array.markers.at(i).header.stamp = ros::Time::now();
    position_marker_array.markers.at(i).ns = leg_name;
    position_marker_array.markers.at(i).id = i;
    position_marker_array.markers.at(i).action = visualization_msgs::Marker::ADD;
    position_marker_array.markers.at(i).lifetime = ros::Duration(t_swing);

    // Foot positions as points
    position_marker_array.markers.at(i).type = visualization_msgs::Marker::SPHERE;
    position_marker_array.markers.at(i).pose.position.x = foot_state.position(0);
    position_marker_array.markers.at(i).pose.position.y = foot_state.position(1);
    position_marker_array.markers.at(i).pose.position.z = foot_state.position(2);
    position_marker_array.markers.at(i).pose.orientation.w = 1.0;
    position_marker_array.markers.at(i).scale.x = 0.01;
    position_marker_array.markers.at(i).scale.y = 0.01;
    position_marker_array.markers.at(i).scale.z = 0.01;

    if (leg_name == "FL" || leg_name == "RR")
    {
      position_marker_array.markers.at(i).color.r = 1.0;
      position_marker_array.markers.at(i).color.g = 0.0;
      position_marker_array.markers.at(i).color.b = 0.0;
      position_marker_array.markers.at(i).color.a = 1.0;
    }
    else
    {
      position_marker_array.markers.at(i).color.r = 0.0;
      position_marker_array.markers.at(i).color.g = 0.0;
      position_marker_array.markers.at(i).color.b = 1.0;
      position_marker_array.markers.at(i).color.a = 1.0;
    }

    phase += dt;
  }

  return position_marker_array;
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

  // Lookup TF from base to foot when leg is in stance phase
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Broadcast post of robot in world frame
  tf2_ros::TransformBroadcaster tf_broadcaster;

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
  std::vector<double> angular_velocity = { 0.0, 0.0, 0.0 };
  pnh.getParam("robot_state/position", position);
  pnh.getParam("robot_state/orientation", orientation);
  pnh.getParam("robot_state/linear_velocity", linear_velocity);
  pnh.getParam("robot_state/angular_velocity", angular_velocity);

  const Quaternion quat_wb = math::Quaternion(orientation.at(3), orientation.at(0),
                                              orientation.at(1), orientation.at(2));
  const mat33 Rwb = quat_wb.matrix();
  const vec x(position);
  const vec xdot(linear_velocity);
  const vec w(angular_velocity);

  geometry_msgs::TransformStamped T_world_base;
  T_world_base.header.frame_id = "world";
  T_world_base.child_frame_id = base_link_name;
  T_world_base.transform.translation.x = x(0);
  T_world_base.transform.translation.y = x(1);
  T_world_base.transform.translation.z = x(2);
  T_world_base.transform.rotation.x = quat_wb.x();
  T_world_base.transform.rotation.y = quat_wb.y();
  T_world_base.transform.rotation.z = quat_wb.z();
  T_world_base.transform.rotation.w = quat_wb.w();

  // Desired robot state in world frame
  std::vector<double> linear_velocity_desired = { 0.0, 0.0, 0.0 };
  std::vector<double> angular_velocity_desired = { 0.0, 0.0, 0.0 };
  pnh.getParam("robot_cmd/linear_velocity", linear_velocity_desired);
  pnh.getParam("robot_cmd/angular_velocity", angular_velocity_desired);

  const vec xdot_d(linear_velocity_desired);
  const vec w_d(angular_velocity_desired);

  // TODO: fix const in QuadrupedKinematics
  const QuadrupedKinematics kinematics;  // FK and IK

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

  ros::Rate rate(5.0);
  while (nh.ok())
  {
    ros::spinOnce();

    // Start planning leg swing trajectories
    const GaitMap gait_map = gait_scheduler.schedule();

    // std::cout << "New loop \n";
    // for (const auto& [key, value] : gait_map)
    // {
    //   std::cout << key << " = " << value.first << " " << value.second << "\n";
    // }

    // Foot positions of all the feet
    FootholdMap foot_map;

    // Publish joint states
    for (const auto& [leg_name, leg_state] : gait_map)
    {
      sensor_msgs::JointState joint_states_msg;

      if (leg_state.first == LegState::swing)
      {
        FootState foot_state =
            foot_traj_manager.referenceState(leg_name, gait_map.at(leg_name).second);

        foot_map.emplace(leg_name, foot_state.position);

        // transform foot position into body frame for IK
        foot_state.position = inv(Rwb) * foot_state.position - x;

        const vec3 q = kinematics.legInverseKinematics(leg_name, foot_state.position);

        joint_states_msg.header.stamp = ros::Time::now();
        joint_states_msg.name = leg_joints_name_map.at(leg_name);
        joint_states_msg.position = arma::conv_to<std::vector<double>>::from(q);
        joint_state_pub.publish(joint_states_msg);
      }

      else
      {
        // Move leg back to initial joints position when in stance
        joint_states_msg.header.stamp = ros::Time::now();
        joint_states_msg.name = leg_joints_name_map.at(leg_name);
        joint_states_msg.position = leg_joints_init_positions_map.at(leg_name);
        joint_state_pub.publish(joint_states_msg);

        const vec q(leg_joints_init_positions_map.at(leg_name));
        vec3 ft_p = kinematics.forwardKinematics(leg_name, q);

        // transform foot position into world
        ft_p = Rwb * ft_p + x;

        foot_map.emplace(leg_name, ft_p);
      }
    }

    // Plan footholds
    const auto foothold_plan =
        foothold_planner.positions(t_stance, Rwb, x, xdot, w, xdot_d, foot_map, gait_map);

    const bool new_footholds = std::get<bool>(foothold_plan);
    const FootholdMap foothold_final_map = std::get<FootholdMap>(foothold_plan);

    // Foot reference states
    FootStateMap foot_states_map;

    // Check if foothold planning happened
    if (!new_footholds)
    {
      // No planning just update reference foot states
      foot_states_map = foot_traj_manager.referenceStates(gait_map);
    }

    else
    {
      // Foot trajectory position only boundary conditions
      FootTrajBoundsMap foot_traj_map;
      for (const auto& [leg_name, p_final] : foothold_final_map)
      {
        foot_traj_map.emplace(leg_name,
                              FootTrajBounds(foothold_start_map.at(leg_name), p_final));
      }

      // Will generate foot trajectories
      foot_states_map = foot_traj_manager.referenceStates(gait_map, foot_traj_map);

      // Visualize foot trajectories
      for (const auto& leg : foothold_final_map)
      {
        const auto traj_marker_message =
            footTrajViz(foot_traj_manager, leg.first, stance_phase, t_swing);

        foot_traj_position_pub.publish(traj_marker_message);
      }
    }

    // Broadcast robot pose
    T_world_base.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(T_world_base);

    // rate.sleep();
  }

  ros::shutdown();
  return 0;
}
