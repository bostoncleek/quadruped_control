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
                                   const vec3& xdot, const vec3& xdot_d,
                                   const vec3& w_d, const GaitMap& gait_map) const
{
  FootholdMap foothold_map;
  for (const auto& [leg_name, leg_state] : gait_map)
  {
    if (leg_state.first == LegState::swing)
    {
      const vec3 fp = singleFoot(t_stance, Rwb, x, xdot, xdot_d, w_d, leg_name);
      foothold_map.emplace(leg_name, fp);
    }
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

  const vec3 p_centrifugal =
      0.5 * std::sqrt(x(2) / g_) * arma::cross(xdot, w_d);  // based on linear inverted pendulum

  // Project on to ground plane (z = 0)
  vec3 foothold = p_hip + p_symmetry + p_centrifugal;
  foothold(2) = 0;

  return foothold;
}

}  // namespace quadruped_controller
