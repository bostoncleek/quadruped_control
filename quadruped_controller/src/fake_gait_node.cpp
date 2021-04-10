/**
 * @file fake_gait_node.cpp
 * @author Boston Cleek
 * @date 2021-04-7
 * @brief fake walking
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
#include <tf2_ros/transform_listener.h>

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

static const std::string LOGNAME = "Fake Gait";

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
    marker_array.markers.at(i).type = visualization_msgs::Marker::SPHERE;
    marker_array.markers.at(i).action = visualization_msgs::Marker::ADD;
    marker_array.markers.at(i).pose.position.x = foot_state.position(0);
    marker_array.markers.at(i).pose.position.y = foot_state.position(1);
    marker_array.markers.at(i).pose.position.z = foot_state.position(2);
    marker_array.markers.at(i).pose.orientation.w = 1.0;
    marker_array.markers.at(i).scale.x = 0.01;
    marker_array.markers.at(i).scale.y = 0.01;
    marker_array.markers.at(i).scale.z = 0.01;

    marker_array.markers.at(i).lifetime = ros::Duration(t_swing);  // TODO: add swing time

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_gait");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Publisher foot_traj_pub =
      nh.advertise<visualization_msgs::MarkerArray>("foot_trajectory", 1);

  ros::Publisher joint_state_pub =
      nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // ros::Publisher foot_hold_pub =
  //     nh.advertise<visualization_msgs::MarkerArray>("foot_hold", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  const auto t_stance = 0.5;  // (s)
  const auto t_swing = 0.5;   // (s)
  const auto height = 0.08;   // max foot height (m)
  const auto stance_phase = t_stance / (t_swing + t_stance);

  // Robot state in world frame
  const mat33 Rwb = arma::eye(3, 3);
  const vec3 x = { 0.0, 0.0, 0.442 };  // standing height is 0.26
  const vec3 xdot = { 100.5, 0.0, 0.0 };
  const vec3 xdot_d = { 0.0, 0.0, 0.0 };
  const vec3 w_d = { 0.0, 0.0, 0.0 };

  // Foot start positions in world frame
  FootholdMap foothold_start_map;
  foothold_start_map.emplace("RL", vec3{ -0.1960, 0.127, 0 });
  foothold_start_map.emplace("FL", vec3{ 0.1960, 0.127, 0 });
  foothold_start_map.emplace("RR", vec3{ -0.1960, -0.127, 0 });
  foothold_start_map.emplace("FR", vec3{ 0.1960, -0.127, 0 });

  // TODO: fix const in QuadrupedKinematics
  QuadrupedKinematics kinematics;  // FK and IK
  const FootPlanner foothold_planner;  // foothold planner
  const FootTrajectoryManager foot_traj_manager(height, t_swing,
                                                t_stance);  // foot trajectories

  const vec phase_offset = { 0.0, 0.5, 0.5, 0.0 };                      // [RL FL RR FR]
  const GaitScheduler gait_scheduler(t_swing, t_stance, phase_offset);  // gait schedule
  gait_scheduler.start();

  ros::Rate rate(200);
  while (nh.ok())
  {
    const GaitMap gait_map = gait_scheduler.schedule();

    // Plan footholds
    const FootholdMap foothold_final_map =
        foothold_planner.positions(t_stance, Rwb, x, xdot, xdot_d, w_d, gait_map);

    // Foot reference states
    FootStateMap foot_states_map;

    // Check if foothold planning happened
    if (foothold_final_map.empty())
    {
      // ROS_INFO_STREAM_NAMED(LOGNAME, "Get reference foot states");
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
        // p_final.print(leg_name);
      }

      // ROS_INFO_STREAM_NAMED(LOGNAME, "Replanning foot trajectories");
      // Will generate foot trajectories
      foot_states_map = foot_traj_manager.referenceStates(gait_map, foot_traj_map);

      // Visualize foot trajectories
      for (const auto& leg : foothold_final_map)
      {
        visualization_msgs::MarkerArray marker_msg =
                  footTrajViz(foot_traj_manager, leg.first, stance_phase, t_swing);

        foot_traj_pub.publish(marker_msg);
      }
    }

    if (gait_map.at("FR").first == LegState::swing)
    {
      FootState FR_foot = foot_traj_manager.referenceState("FR", gait_map.at("FR").second);
      vec3 trans_fr = { 0.196, -0.050, 0.0 };
      FR_foot.position = inv(Rwb) * FR_foot.position - x;      
      const vec3 q = kinematics.legInverseKinematics("FR", FR_foot.position);

      sensor_msgs::JointState joint_states_msg;
      joint_states_msg.header.stamp = ros::Time::now();
      
      joint_states_msg.name.emplace_back("FR_hip_joint");
      joint_states_msg.name.emplace_back("FR_thigh_joint");
      joint_states_msg.name.emplace_back("FR_calf_joint");

      joint_states_msg.position.emplace_back(q(0));
      joint_states_msg.position.emplace_back(q(1));
      joint_states_msg.position.emplace_back(q(2));

      joint_state_pub.publish(joint_states_msg);
    }

    if (gait_map.at("FL").first == LegState::swing)
    {
      FootState FL_foot = foot_traj_manager.referenceState("FL", gait_map.at("FL").second);
      FL_foot.position = inv(Rwb) * FL_foot.position - x;      
      const vec3 q = kinematics.legInverseKinematics("FL", FL_foot.position);

      sensor_msgs::JointState joint_states_msg;
      joint_states_msg.header.stamp = ros::Time::now();
      
      joint_states_msg.name.emplace_back("FL_hip_joint");
      joint_states_msg.name.emplace_back("FL_thigh_joint");
      joint_states_msg.name.emplace_back("FL_calf_joint");

      joint_states_msg.position.emplace_back(q(0));
      joint_states_msg.position.emplace_back(q(1));
      joint_states_msg.position.emplace_back(q(2));

      joint_state_pub.publish(joint_states_msg);
    }

    if (gait_map.at("RR").first == LegState::swing)
    {
      FootState RR_foot = foot_traj_manager.referenceState("RR", gait_map.at("RR").second);
      RR_foot.position = inv(Rwb) * RR_foot.position - x;      
      const vec3 q = kinematics.legInverseKinematics("RR", RR_foot.position);

      sensor_msgs::JointState joint_states_msg;
      joint_states_msg.header.stamp = ros::Time::now();
      
      joint_states_msg.name.emplace_back("RR_hip_joint");
      joint_states_msg.name.emplace_back("RR_thigh_joint");
      joint_states_msg.name.emplace_back("RR_calf_joint");

      joint_states_msg.position.emplace_back(q(0));
      joint_states_msg.position.emplace_back(q(1));
      joint_states_msg.position.emplace_back(q(2));

      joint_state_pub.publish(joint_states_msg);
    }

    if (gait_map.at("RL").first == LegState::swing)
    {
      FootState RL_foot = foot_traj_manager.referenceState("RL", gait_map.at("RL").second);
      RL_foot.position = inv(Rwb) * RL_foot.position - x;      
      const vec3 q = kinematics.legInverseKinematics("RL", RL_foot.position);

      sensor_msgs::JointState joint_states_msg;
      joint_states_msg.header.stamp = ros::Time::now();
      
      joint_states_msg.name.emplace_back("RL_hip_joint");
      joint_states_msg.name.emplace_back("RL_thigh_joint");
      joint_states_msg.name.emplace_back("RL_calf_joint");

      joint_states_msg.position.emplace_back(q(0));
      joint_states_msg.position.emplace_back(q(1));
      joint_states_msg.position.emplace_back(q(2));

      joint_state_pub.publish(joint_states_msg);
    }


    // rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
