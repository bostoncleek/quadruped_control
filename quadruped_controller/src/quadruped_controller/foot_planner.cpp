/**
 * @file foothold.cpp
 * @date 2021-04-3
 * @author Boston Cleek
 * @brief foothold planner
 */

#include <cmath>
#include <quadruped_controller/foot_planner.hpp>

namespace quadruped_controller
{
FootPlanner::FootPlanner() : g_(9.81)
{
  // TODO: Load all these in
  k_ = 0.03;

  // Vector from base to hip
  const auto xbh = 0.196;
  const auto ybh = 0.050;
  const auto zbh = 0.0;

  // Base to each hip
  const vec3 trans_rl = { -xbh, ybh, zbh };
  const vec3 trans_fl = { xbh, ybh, zbh };

  const vec3 trans_rr = { -xbh, -ybh, zbh };
  const vec3 trans_fr = { xbh, -ybh, zbh };

  hip_map_.emplace("RL", trans_rl);
  hip_map_.emplace("FL", trans_fl);
  hip_map_.emplace("RR", trans_rr);
  hip_map_.emplace("FR", trans_fr);
}

FootholdMap FootPlanner::positions(double t_stance, const mat33& Rwb, const vec3& x,
                                   const vec3& xdot, const vec3& xdot_d, const vec3& w_d,
                                   const GaitMap& gait_map) const
{
  FootholdMap foothold_map;

  // Check for legs that we may need to replan footholds for
  // and update the current states of the legs.
  const std::vector<std::string> plan_legs = updateStates(gait_map);

  // No need to replan foot holds
  if (plan_legs.empty())
  {
    return foothold_map;
  }

  // Plan footholds
  for (const auto& leg_name : plan_legs)
  {
    const vec3 foothold = singleFoot(t_stance, Rwb, x, xdot, xdot_d, w_d, leg_name);
    foothold_map.emplace(leg_name, foothold);
  }

  return foothold_map;
}

vec3 FootPlanner::singleFoot(double t_stance, const mat33& Rwb, const vec3& x,
                             const vec3& xdot, const vec3& xdot_d, const vec3& w_d,
                             const std::string& leg_name) const
{
  const vec3 p_hip = x + Rwb * hip_map_.at(leg_name);  // hip position in world frame

  const vec3 p_symmetry =
      (t_stance / 2.0) * xdot + k_ * (xdot - xdot_d);  // Raibert heuristic

  const vec3 p_centrifugal = 0.5 * std::sqrt(x(2) / g_) *
                             arma::cross(xdot, w_d);  // based on linear inverted pendulum

  // Project on to ground plane (z = 0)
  vec3 foothold = p_hip + p_symmetry + p_centrifugal;
  foothold(2) = 0;

  return foothold;
}

std::vector<std::string> FootPlanner::updateStates(const GaitMap& gait_map) const
{
  // replan for these legs if needed
  std::vector<std::string> plan_legs;

  // No legs in state_map_
  if (state_map_.empty())
  {
    for (const auto& [leg_name, leg_state] : gait_map)
    {
      // Add leg to state_map_
      state_map_.emplace(leg_name, leg_state.first);

      // plan foot holds for legs in swing phase
      if (leg_state.first == LegState::swing)
      {
        plan_legs.emplace_back(leg_name);
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
        // Replan
        plan_legs.emplace_back(leg_name);

        // Update state
        state_map_.at(leg_name) = leg_state.first;
      }

      else if ((state_map_.at(leg_name) == LegState::swing) &&
               (leg_state.first == LegState::stance))
      {
        // Update state
        state_map_.at(leg_name) = leg_state.first;
      }
    }
  }

  return plan_legs;
}

}  // namespace quadruped_controller
