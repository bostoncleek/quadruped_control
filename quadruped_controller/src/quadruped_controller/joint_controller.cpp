/**
 * @file pid_controller.cpp
 * @date 2021-02-21
 * @author Boston Cleek
 * @brief Joint PD control
 */

#include <quadruped_controller/joint_controller.hpp>
#include <quadruped_controller/math/numerics.hpp>

namespace quadruped_controller
{
using math::normalize_angle_2PI;
using math::normalize_angle_PI;

JointController::JointController(const vec3& kff, const vec3& kp, const vec3& kd)
  : kff_(kff), kp_(kp), kd_(kd)
{
}

TorqueMap JointController::control(const JointStatesMap& joints_ref_map,
                                   const JointStatesMap& joints_map) const
{
  TorqueMap torque_map;
  for (const auto& [leg_name, joint_ref_states] : joints_ref_map)
  {
    const vec3 q_error_normalized = normalize_angle_2PI(joint_ref_states.q) -
                                    normalize_angle_2PI(joints_map.at(leg_name).q);

    const vec3 q_error = normalize_angle_PI(q_error_normalized);

    const vec3 qdot_error = joint_ref_states.qdot - joints_map.at(leg_name).qdot;
    const vec3 tau = kp_ % q_error + kd_ % qdot_error + kff_;

    torque_map.emplace(leg_name, tau);
  }

  return torque_map;
}
}  // namespace quadruped_controller
