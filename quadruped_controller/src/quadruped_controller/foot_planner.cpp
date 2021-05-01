/**
 * @file foothold.cpp
 * @date 2021-04-3
 * @author Boston Cleek
 * @brief foothold planner
 */

// C++
#include <cmath>

// ROS
#include <ros/console.h>

// Quadruped Control
#include <quadruped_controller/foot_planner.hpp>
#include <quadruped_controller/math/rigid3d.hpp>

namespace quadruped_controller
{
static const std::string LOGNAME = "foothold_planner";

FootPlanner::FootPlanner() : g_(9.81)
{
  // TODO: Load all these in
  k_ = 0.01;

  // distance from base to thigh
  const auto xbt = 0.196;
  const auto ybt = 0.127;
  const auto zbt = 0.0;

  // Base to each hip
  const vec3 trans_rl = { -xbt, ybt, zbt };
  const vec3 trans_fl = { xbt, ybt, zbt };

  const vec3 trans_rr = { -xbt, -ybt, zbt };
  const vec3 trans_fr = { xbt, -ybt, zbt };

  hip_map_.emplace("RL", trans_rl);
  hip_map_.emplace("FL", trans_fl);
  hip_map_.emplace("RR", trans_rr);
  hip_map_.emplace("FR", trans_fr);
}

tuple<bool, FootholdMap> FootPlanner::positions(double t_stance, const mat33& Rwb,
                                                const vec3& x, const vec3& xdot,
                                                const vec3& w, const vec3& xdot_d,
                                                const FootholdMap& foot_holds,
                                                const GaitMap& gait_map) const
{
  // Check for legs that we may need to replan footholds for
  // and update the current states of the legs.
  const std::vector<std::string> plan_legs = updateStates(gait_map);

  // No need to replan foot holds
  if (plan_legs.empty())
  {
    ROS_DEBUG_NAMED(LOGNAME, "No need to replan footholds");
    return std::make_tuple(false, FootholdMap());
  }

  // Plan footholds
  FootholdMap foothold_map;
  for (const auto& leg_name : plan_legs)
  {
    ROS_DEBUG_NAMED(LOGNAME, "Finished foot step planning for leg: %s", leg_name.c_str());
    const vec foothold_actual = foot_holds.at(leg_name);  // in body frame
    const vec3 foothold =
        singleFoot(t_stance, Rwb, x, xdot, w, xdot_d, foothold_actual, leg_name);
    foothold_map.emplace(leg_name, foothold);
  }

  return std::make_tuple(true, foothold_map);
}

vec3 FootPlanner::singleFoot(double t_stance, const mat33& Rwb, const vec3& x,
                             const vec3& xdot, const vec3& w, const vec3& xdot_d,
                             const vec3& foot_position, const std::string& leg_name) const
{
  // thigh position in world frame
  const vec3 p_thigh = Rwb * hip_map_.at(leg_name) + x;

  // vector from COM to foot in world frame
  // pcom_foot = foot_position - x_com = (Rwb * foot_position + x_com) - x_com = Rwb * foot_position
  const vec3 pcom_foot = Rwb * foot_position;

  // Tangential velocity
  const vec3 tang_vel = arma::cross(w, pcom_foot);

  // Raibert heuristic
  const vec3 p_linear = (t_stance / 2.0) * xdot + k_ * (xdot - xdot_d);
  const vec3 p_tangent = (t_stance / 2.0) * tang_vel;

  // based on linear inverted pendulum
  const vec3 p_lip = 0.5 * std::sqrt(x(2) / g_) * xdot;

  // Project on to ground plane (z = 0)
  vec3 foothold = p_thigh + p_linear + p_tangent + p_lip;

  // TODO: get TF from world to foot link
  foothold(2) = 0.0;

  return foothold;
}

std::vector<std::string> FootPlanner::updateStates(const GaitMap& gait_map) const
{
  // replan for these legs if needed
  std::vector<std::string> plan_legs;

  // No legs in state_map_
  if (state_map_.empty())
  {
    ROS_DEBUG_NAMED(LOGNAME, "Populating leg states for foothold planning");

    for (const auto& [leg_name, leg_state] : gait_map)
    {
      // Add leg to state map
      state_map_.emplace(leg_name, leg_state.first);

      // plan footholds for legs in swing phase
      if (leg_state.first == LegState::swing)
      {
        ROS_DEBUG_NAMED(LOGNAME, "Scheduled to plan foothold for leg: %s",
                        leg_name.c_str());
        plan_legs.emplace_back(leg_name);
      }

      else
      {
        ROS_DEBUG_NAMED(LOGNAME, "leg in stance: %s", leg_name.c_str());
      }
    }
  }

  else
  {
    // Replan if leg was in stance but now in swing
    for (const auto& [leg_name, leg_state] : gait_map)
    {
      if ((state_map_.at(leg_name) == LegState::stance) &&
          (leg_state.first == LegState::swing))
      {
        ROS_DEBUG_NAMED(LOGNAME, "Scheduled to plan foothold for leg: %s",
                        leg_name.c_str());

        // Replan
        plan_legs.emplace_back(leg_name);
      }

      // Update state
      state_map_.at(leg_name) = leg_state.first;
    }
  }

  return plan_legs;
}

}  // namespace quadruped_controller
