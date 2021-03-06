/**
 * @file balance_controller.cpp
 * @date 2021-03-04
 * @author Boston Cleek
 * @brief Force balance controller
 */

#include <quadruped_controller/balance_controller.hpp>

/*
References:
  [R1] M. Focchi, A. del Prete, I. Havoutis, R. Featherstone, D. G. Caldwell,
      and C. Semini. High-slope terrain locomotion for torque-controlled quadruped
      robots. Autonomous Robots, 2016.

  [R2] R. M. Murray, S. S. Sastry, and L. Zexiang. A Mathematical Introduction
  to Robotic Manipulation. CRC Press, Inc., Boca Raton, FL, USA, 1st edition, 1994.
*/

namespace quadruped_controller
{
BalanceController::BalanceController(double mass, const mat& Ib, const vec& kp_p,
                                     const vec& kd_p, const vec& kp_w, const vec& kd_w)
  : mass_(mass)
  , Ib_(Ib)
  , kp_p_(kp_p)
  , kd_p_(kd_p)
  , kp_w_(kp_w)
  , kd_w_(kd_w)
  , g_({ 0., 0., -9.81 })
{
}


vec BalanceController::control(const mat& ft_p, const mat& Rb, const mat& Rb_d,
                               const vec& x, const vec& xdot, const vec& w,
                               const vec& x_d, const vec& xdot_d, const vec& w_d)
{
  // IMPORTANT: Ground reaction forces are in world frame
  // PD control on COM position and orientation
  // [R1] Eq(3)
  const vec xddot_d = kp_p_ % (x_d - x) + kd_p_ % (xdot_d - xdot);
  // xddot_d.print("xddot_d");

  // [R2] Proposition 2.5 and [R1] Eq(4)
  const Rotation3d R_error(Rb_d * Rb.t());
  const vec wdot_d = kp_w_ % R_error.angleAxisTotal() + kd_w_ % (w_d - w);
  wdot_d.print("wdot_d");

  // [R1] Eq(5) Linear Euler's single rigid boyd dynamics
  const auto euler_dyn = constructDynamics(ft_p, Rb, x, xddot_d, wdot_d);


  // Ground reaction forces in body frame
  vec fb(12, arma::fill::zeros);
  return fb;
}


tuple<mat, vec> BalanceController::constructDynamics(const mat& ft_p, const mat& Rb,
                                                     const vec& x, const vec& xddot_d,
                                                     const vec& wdot_d) const
{
  // Vector from COM to each foot position
  const mat x_ft_p = ft_p.each_col() - x;

  // Moment of Interia in world frame
  // Iw = Rwb * Ib * Rwb^T
  const mat Iw = Rb * Ib_ * Rb.t();
  Iw.print("Iw");

  mat A(6, 12, arma::fill::zeros);
  A.submat(0, 0, 2, 2) = eye(3, 3);
  A.submat(0, 3, 2, 5) = eye(3, 3);
  A.submat(0, 6, 2, 8) = eye(3, 3);
  A.submat(0, 9, 2, 11) = eye(3, 3);

  A.submat(3, 0, 5, 2) = skew_symmetric(x_ft_p.col(0));
  A.submat(3, 3, 5, 5) = skew_symmetric(x_ft_p.col(1));
  A.submat(3, 6, 5, 8) = skew_symmetric(x_ft_p.col(2));
  A.submat(3, 9, 5, 11) = skew_symmetric(x_ft_p.col(3));

  // A.print("Euler RBD: A");

  vec b(6, arma::fill::zeros);
  b.rows(0, 2) = mass_ * (xddot_d + g_);
  b.rows(3, 5) = Iw * wdot_d;

  // b.print("Euler RBD: b");

  return std::make_tuple(A, b);
}


}  // namespace quadruped_controller
