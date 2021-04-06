/**
 * @file foothold.hpp
 * @date 2021-04-3
 * @author Boston Cleek
 * @brief foothold planner
 */
#ifndef FOOTHOLD_HPP
#define FOOTHOLD_HPP

#include <quadruped_controller/types.hpp>

namespace quadruped_controller
{
class FootPlanner
{
public:
  FootPlanner();

  FootholdMap positions(double t_stance, const mat33& Rwb, const vec3& x,
                        const vec3& xdot, const vec3& xdot_d, const vec3& w_d,
                        const GaitMap& gait_map) const;

  vec3 singleFoot(double t_stance, const mat33& Rwb, const vec3& x, const vec3& xdot,
                  const vec3& xdot_d, const vec3& w_d, const std::string& leg_name) const;

private:
  double g_;                             // gravitaional magnitude (m/s^2)
  double k_;                             // feedback gain
  std::map<std::string, vec3> hip_map_;  // map leg name to vector from com to hip
};
}  // namespace quadruped_controller
#endif
