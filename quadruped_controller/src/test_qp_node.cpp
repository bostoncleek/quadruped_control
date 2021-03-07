// C++
#include <iostream>
#include <utility>

// ROS
#include <ros/ros.h>

// Linear Algebra
#include <armadillo>

#include <quadruped_controller/balance_controller.hpp>

using arma::eye;
using arma::mat;
using arma::vec;

using quadruped_controller::BalanceController;
using quadruped_controller::rigid3d::Quaternion;
using quadruped_controller::rigid3d::Rotation3d;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "commander");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  const auto mu = 0.8;    // Coefficient of friction
  const auto mass = 9.0;  // total mass

  const auto fzmin = 10.0;
  const auto fzmax = 160.0;

  // TODO: update to total inertia of robot?
  mat Ib(3, 3, arma::fill::zeros);  // inertia of base
  Ib(0, 0) = 0.011253;
  Ib(1, 1) = 0.036203;
  Ib(2, 2) = 0.042673;

  mat S = eye(6, 6);

  const vec kp_p = { 1., 1., 1. };
  const vec kp_w = kp_p;

  const vec kd_p = { 1., 1., 1. };
  const vec kd_w = kd_p;

  // Rotations from world to body
  Quaternion qwb_d;  //(0.966, 0., 0., 0.259);  // desired
  Rotation3d Rwb_d(qwb_d);

  Quaternion qwb;  // actual
  Rotation3d Rwb(qwb);

  // COM in world frame
  vec x(3, arma::fill::zeros);
  vec xdot(3, arma::fill::zeros);
  vec w(3, arma::fill::zeros);

  vec x_d = { 0., 0., 0.2 };
  vec xdot_d(3, arma::fill::zeros);
  vec w_d(3, arma::fill::zeros);

  // Acutall foot positions in world frame
  // rows: FL, FR, RL, RR
  mat ft_p(3, 4, arma::fill::zeros);
  ft_p(0, 0) = 0.220;
  ft_p(1, 0) = 0.136;
  ft_p(2, 0) = -0.029;

  ft_p(0, 1) = 0.220;
  ft_p(1, 1) = -0.136;
  ft_p(2, 1) = -0.029;

  ft_p(0, 2) = -0.172;
  ft_p(1, 2) = 0.136;
  ft_p(2, 2) = -0.029;

  ft_p(0, 3) = -0.172;
  ft_p(1, 3) = -0.136;
  ft_p(2, 3) = -0.029;

  // mat A_test(6,6, arma::fill::ones);
  // mat S_test(6,6, arma::fill::ones);
  // mat H = A_test * S_test;
  // H.print("H");

  BalanceController balance_controller(mu, mass, fzmin, fzmax, Ib, S, kp_p, kd_p, kp_w,
                                       kd_w);

  vec fb1 = balance_controller.control(ft_p, Rwb.rotationMatrix(), Rwb_d.rotationMatrix(),
                                       x, xdot, w, x_d, xdot_d, w_d);

  fb1.print("fb1");

  x_d = { 0., 0., 0.1 };

  vec fb2 = balance_controller.control(ft_p, Rwb.rotationMatrix(), Rwb_d.rotationMatrix(),
                                       x, xdot, w, x_d, xdot_d, w_d);

  fb2.print("fb2");

  return 0;
}
