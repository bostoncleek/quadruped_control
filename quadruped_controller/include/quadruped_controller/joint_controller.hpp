/**
 * @file pid_controller.hpp
 * @date 2021-02-21
 * @author Boston Cleek
 * @brief PID control
 */
#ifndef JOINT_CONTROLLER_HPP
#define JOINT_CONTROLLER_HPP

// Linear Algebra
#include <armadillo>

#include <quadruped_controller/types.hpp>

namespace quadruped_controller
{
class JointController
{
public:
  JointController(const vec3& kff, const vec3& kp, const vec3& kd);

  TorqueMap control(const JointStatesMap& joints_ref_map,
                    const JointStatesMap& joints_map) const;

private:
  vec3 kff_;      // FF gains
  vec3 kp_, kd_;  // PD gains
};
}  // namespace quadruped_controller
#endif
