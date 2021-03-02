/**
 * @file pid_controller.hpp
 * @date 2021-02-21
 * @author Boston Cleek
 * @brief PID control
 */
#ifndef JOINT_CONTROLLER_HPP
#define JOINT_CONTROLLER_HPP

#include <armadillo>

namespace quadruped_controller
{
using arma::vec;

class JointController
{
public:
  JointController(const vec& kp, const vec& kd);

  JointController(const vec& kp, const vec& kd, const vec& tau_ff, double tau_min,
                  double tau_max);

  vec control(const vec& q, const vec& qd, const vec& v, const vec& vd);

  vec positionError()
  {
    return q_error_;
  }

  vec velocityError()
  {
    return v_error_;
  }

private:
  bool tau_lim_;
  double tau_min_, tau_max_;
  vec tau_ff_;
  vec kp_, kd_;            // joint position and angular velocity gains
  vec q_error_, v_error_;  // joint position and angular velocity error
};
}  // namespace quadruped_controller
#endif
