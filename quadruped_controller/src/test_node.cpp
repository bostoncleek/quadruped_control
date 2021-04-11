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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ScheduledPhasesMap schedule_map;
  schedule_map.emplace("RL", LegScheduledPhases{ 0.0, 0.5, 0.5, 1.0 });
  schedule_map.emplace("FL", LegScheduledPhases{ 0.5, 1.0, 0.0, 0.5 });
  schedule_map.emplace("RR", LegScheduledPhases{ 0.5, 1.0, 0.0, 0.5 });
  schedule_map.emplace("FR", LegScheduledPhases{ 0.0, 0.5, 0.5, 1.0 });

  GaitMap gait_map;
  gait_map.emplace("RL", std::make_pair(LegState::stance, 0.2));
  gait_map.emplace("FL", std::make_pair(LegState::swing, 0.6));
  gait_map.emplace("RR", std::make_pair(LegState::swing, 0.6));
  gait_map.emplace("FR", std::make_pair(LegState::stance, 0.2));

  FootholdMap foot_map;
  foot_map.emplace("RL", vec3{ -0.196, 0.050, 0.0 });
  foot_map.emplace("FL", vec3{ 0.196, 0.050, 0.0 });
  foot_map.emplace("RR", vec3{ -0.196, -0.050, 0.0 });
  foot_map.emplace("FR", vec3{ 0.196, -0.050, 0.0 });

  SupportPolygon support_poylgon;
  support_poylgon.position(schedule_map, foot_map, gait_map).print();

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // QuadrupedKinematics kinematics;

  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);

  // ros::Rate rate(10);
  // while (nh.ok())
  // {
  //   // const auto l1 = 0.077;
  //   // const auto l2 = 0.211;
  //   // const auto l3 = 0.230;

  //   try
  //   {
  //     const geometry_msgs::TransformStamped T_hip_foot =
  //         tfBuffer.lookupTransform("RR_hip0", "RR_foot", ros::Time(0));

  //     vec3 FR_foot = { T_hip_foot.transform.translation.x,
  //                      T_hip_foot.transform.translation.y,
  //                      T_hip_foot.transform.translation.z };

  //     kinematics.legInverseKinematics("RR", FR_foot).print("RR IK");
  //   }

  //   catch (tf2::TransformException& ex)
  //   {
  //     ROS_WARN_ONCE("%s", ex.what());
  //   }

  //   rate.sleep();
  // }

  // FootTrajectory foot_traj;

  // vec3 p0 = { 0.0, 0.0, 0.0 };
  // vec3 pf = { 0.1, 0.0, 0.0 };

  // // position at center of trajectory
  // vec3 pc = 0.5 * (p0 + pf);
  // pc(2) = 0.08;

  // if (foot_traj.generateTrajetory(p0, pc, pf))
  // {
  //   FootState state = foot_traj.trackTrajectory(1.0);
  //   state.position.print("p_ref");
  //   state.velocity.print("v_ref");
  // }

  // const auto t_stance = 0.1;  // (s)
  // const auto t_swing = 0.1;   // (s)
  // const auto height = 0.08;

  // FootPlanner foothold_planner;
  // FootTrajectoryManager foot_traj_manager(height, t_swing, t_stance);

  // // Robot state
  // mat33 Rwb = arma::eye(3, 3);
  // vec3 x = { 0.0, 0.0, 0.26 };
  // vec3 xdot = { 0.0, 0.0, 0.0 };
  // vec3 xdot_d = { 0.0, 0.0, 0.0 };
  // vec3 w_d = { 0.0, 0.0, 0.0 };

  // // Foot start positions
  // const vec3 p0_RL = { -0.1960, 0.0500, 0 };
  // const vec3 p0_FL = { 0.1960, 0.0500, 0 };
  // const vec3 p0_RR = { -0.1960, -0.0500, 0 };
  // const vec3 p0_FR = { 0.1960, -0.0500, 0 };

  // GaitMap gait_map;
  // gait_map.emplace("RL", std::make_pair(LegState::stance, 0.2));
  // gait_map.emplace("FL", std::make_pair(LegState::swing, 0.6));
  // gait_map.emplace("RR", std::make_pair(LegState::swing, 0.6));
  // gait_map.emplace("FR", std::make_pair(LegState::stance, 0.2));

  // FootholdMap foothold_map =
  //     foothold_planner.positions(t_stance, Rwb, x, xdot, xdot_d, w_d, gait_map);

  // for (const auto& [leg_name, p_final] : foothold_map)
  // {
  //   p_final.print(leg_name);
  // }

  // FootTrajBoundsMap foot_traj_map;
  // foot_traj_map.emplace("FL", FootTrajBounds(p0_FL, foothold_map.at("FL")));
  // foot_traj_map.emplace("RR", FootTrajBounds(p0_RR, foothold_map.at("RR")));

  // FootStateMap foot_states_map =
  //     foot_traj_manager.referenceStates(gait_map, foot_traj_map);

  // for (const auto& [leg_name, foot_state] : foot_states_map)
  // {
  //   foot_state.position.print(leg_name + ": position");
  //   foot_state.velocity.print(leg_name + ": velocity");
  // }

  // GaitMap gait_map2;
  // gait_map2.emplace("RL", std::make_pair(LegState::swing, 0.2));
  // gait_map2.emplace("FL", std::make_pair(LegState::stance, 0.6));
  // gait_map2.emplace("RR", std::make_pair(LegState::stance, 0.6));
  // gait_map2.emplace("FR", std::make_pair(LegState::swing, 0.2));

  // foot_states_map = foot_traj_manager.referenceStates(gait_map2);

  // for (const auto& [leg_name, foot_state] : foot_states_map)
  // {
  //   foot_state.position.print(leg_name + ": position");
  //   foot_state.velocity.print(leg_name + ": velocity");
  // }

  // [RL FL RR FR]
  // const vec phase_offset = { 0.0, 0.5, 0.5, 0.0 };
  // const GaitScheduler gait_scheduler(t_swing, t_stance, phase_offset);
  // gait_scheduler.start();

  // // auto stance_detected_10 = std::chrono::steady_clock::now();
  // // auto swing_detected_10 = std::chrono::steady_clock::now();

  // // auto stance_detected_01 = std::chrono::steady_clock::now();
  // // auto swing_detected_01 = std::chrono::steady_clock::now();

  // // int state = 0;

  // ros::Rate rate(100);
  // while (nh.ok())
  // {
  //   GaitMap gait_map = gait_scheduler.schedule();

  //   // if (gait_map.at("RL") == 1 && state == 0)
  //   // {
  //   //   stance_detected_10 = std::chrono::steady_clock::now();
  //   //   state = 1;

  //   //   stance_detected_01 = std::chrono::steady_clock::now();

  //   //   std::cout << "switch from 0 -> 1: " << std::chrono::duration<double>(stance_detected_01
  //   //   - swing_detected_01).count() << std::endl;
  //   // }

  //   // else if (gait_map.at("RL") == 0 && state == 1)
  //   // {
  //   //   swing_detected_10 = std::chrono::steady_clock::now();
  //   //   state = 0;

  //   //   swing_detected_01 = std::chrono::steady_clock::now();

  //   //   std::cout << "switch from 1 -> 0: " << std::chrono::duration<double>(swing_detected_10
  //   //   - stance_detected_10).count() << std::endl;
  //   // }

  //   std::cout << "---------"
  //             << "\n";
  //   for (const auto& [key, value] : gait_map)
  //   {
  //     std::cout << key << " = " << value.first << " " << value.second << "\n";
  //   }

  //   rate.sleep();
  // }

  ros::waitForShutdown();
  return 0;
}
