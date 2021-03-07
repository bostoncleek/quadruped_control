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
  Quaternion qwb_d(0.966, 0., 0., 0.259);  // desired
  Rotation3d Rwb_d(qwb_d);

  Quaternion qwb;  // actual
  Rotation3d Rwb(qwb);

  // COM in world frame
  vec x(3, arma::fill::zeros);
  vec xdot(3, arma::fill::zeros);
  vec w(3, arma::fill::zeros);

  vec x_d = { 0., 0., 0.1 };
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

  balance_controller.control(ft_p, Rwb.rotationMatrix(), Rwb_d.rotationMatrix(), x, xdot,
                             w, x_d, xdot_d, w_d);


  // ///////////////////////////////
  // // Setup QP
  // int nWSR = 100;  // max working set recalculations
  // int nV = 12;     // variables
  // // int nC = 3; // constraints

  // mat Q = 2.0 * A.t() * A;
  // vec c = -2.0 * A.t() * b;

  // vec lb(12, arma::fill::ones);
  // vec ub(12, arma::fill::ones);
  // lb *= 2.;
  // ub *= 100.;

  // // row wise matrix
  // real_t qp_Q[12 * 12];
  // real_t qp_c[12];

  // copy_to_real_t(Q, qp_Q);
  // copy_to_real_t(c, qp_c);

  // // print_real_t(qp_Q, 12, 12, "qp_Q:");
  // // print_real_t(qp_c, 12, 1, "qp_c:");

  // real_t qp_lb[12];
  // real_t qp_ub[12];

  // copy_to_real_t(lb, qp_lb);
  // copy_to_real_t(ub, qp_ub);

  // // real_t* lbA = nullptr;
  // // real_t* ubA = nullptr;

  // QProblemB problem(nV);

  // // Options options;
  // // options.printLevel = qpOASES::PL_NONE;
  // // // options.PrintLevel = qpOASES::PL_DEBUG_ITER;
  // // problem.setOptions( options );

  // // returnValue retVal = problem.init( H, g, A, lb, ub, lbA, ubA, nWSR );
  // returnValue retVal = problem.init(qp_Q, qp_c, qp_lb, qp_ub, nWSR);
  // std::cout << "Return value from init: " << retVal << "\n";

  // real_t xOpt[12];
  // // // real_t yOpt[2+1];
  // // // real_t cost = problem.getObjVal();
  // problem.getPrimalSolution(xOpt);
  // // // problem.getDualSolution( yOpt );


  // print_real_t(xOpt, 12, 1, "xOpt:");


  // problem.printOptions();


  return 0;
}
