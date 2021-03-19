/**
 * @file pid_controller.cpp
 * @date 2021-02-21
 * @author Boston Cleek
 * @brief PID control
 */

#include <exception>
#include <quadruped_controller/joint_controller.hpp>

namespace quadruped_controller
{
JointController::JointController(const vec& kp, const vec& kd)
  : tau_lim_(false)
  , kp_(kp)
  , kd_(kd)
  , q_error_(kd.size(), arma::fill::zeros)
  , v_error_(kd.size(), arma::fill::zeros)
{
  if (kp.size() != kd.size())
  {
    throw std::invalid_argument("The dimension of kp must equal the dimension of kd");
  }
}

JointController::JointController(const vec& kp, const vec& kd, const vec& tau_ff,
                                 double tau_min, double tau_max)
  : tau_lim_(true)
  , tau_min_(tau_min)
  , tau_max_(tau_max)
  , tau_ff_(tau_ff)
  , kp_(kp)
  , kd_(kd)
  , q_error_(kd.size(), arma::fill::zeros)
  , v_error_(kd.size(), arma::fill::zeros)
{
  if (kp.size() != kd.size())
  {
    throw std::invalid_argument("The dimension of kp must equal the dimension of kd");
  }
}

vec JointController::control(const vec& q, const vec& qd, const vec& v, const vec& vd)
{
  q_error_ = qd - q;
  v_error_ = vd - v;
  vec tau = kp_ % q_error_ + kd_ % v_error_ + tau_ff_;

  if (tau_lim_)
  {
    // std::cout << "Tau limit" << std::endl;
    tau = arma::clamp(tau, tau_min_, tau_max_);
    // tau.print("tau");
  }

  return tau;
}

}  // namespace quadruped_controller