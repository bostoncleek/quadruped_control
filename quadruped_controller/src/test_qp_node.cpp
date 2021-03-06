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


using qpOASES::Options;
using qpOASES::QProblem;
using qpOASES::QProblemB;
using qpOASES::real_t;
using qpOASES::returnValue;

void copy_to_real_t(const vec& source, real_t* target)
{
  for (unsigned int i = 0; i < source.size(); i++)
  {
    target[i] = source(i);
  }
}


void copy_to_real_t(const mat& source, real_t* target)
{
  unsigned int count = 0;
  for (unsigned int i = 0; i < source.n_rows; i++)
  {
    for (unsigned int j = 0; j < source.n_cols; j++)
    {
      target[count] = source(i, j);
      count++;
    }
  }
}


void print_real_t(const real_t* const array, unsigned int n_rows, unsigned int n_cols,
                  const std::string& msg = "")
{
  std::cout << msg << "\n";
  unsigned int count = 0;
  for (unsigned int i = 0; i < n_rows; i++)
  {
    for (unsigned int j = 0; j < n_cols; j++)
    {
      std::cout << array[count] << " ";
      count++;
    }
    std::cout << std::endl;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "commander");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  const double mass = 9;  // total mass

  // TODO: update to total inertia of robot?
  mat Ib(3, 3, arma::fill::zeros);  // inertia of base
  Ib(0, 0) = 0.011253;
  Ib(1, 1) = 0.036203;
  Ib(2, 2) = 0.042673;

  const vec kp_p = { 1., 1., 1. };
  const vec kp_w = kp_p;

  const vec kd_p = { 1., 1., 1. };
  const vec kd_w = kd_p;

  BalanceController balance_controller(mass, Ib, kp_p, kd_p, kp_w, kd_w);


  // Rotations from world to body
  Quaternion qb_d(0.966, 0., 0., 0.259);  // desired
  Rotation3d Rb_d(qb_d);

  Quaternion qb;  // actual
  Rotation3d Rb(qb);

  // Rotation3d R_err(Rb_d.rotationMatrix() * Rb.rotationMatrix().t());
  // auto aa_err = R_err.angleAxis();

  // std::get<vec>(aa_err).print("axis");
  // std::cout << std::get<double>(aa_err) * 180. / M_PI << std::endl;


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

  balance_controller.control(ft_p, Rb.rotationMatrix(), Rb_d.rotationMatrix(), x, xdot, w,
                             x_d, xdot_d, w_d);


  // TODO: add S too optimization
  // TODO: which frame for gravity vector??


  // // Weight matrix
  // mat S = eye(6, 6);

  // // Treating COM the same as the body frame
  // // Centroidal rotational inertia in body frame
  // mat Ig(3, 3, arma::fill::zeros);
  // Ig(0, 0) = 0.011253;
  // Ig(1, 1) = 0.036203;
  // Ig(2, 2) = 0.042673;

  // // mass
  // double m = 9.;

  // // gravity
  // vec g = { 0., 0., -9.81 };

  // // Linear acceleration in body frame
  // vec pddotb = { 0., 0., 1. };

  // // Angular accleration in body frame
  // vec wdotb = { 0., 0., 0.0 };

  // // Positions of each leg in body frame
  // // FL
  // vec p1 = { 0.159, 0.127, -0.305 };

  // // FR
  // vec p2 = { 0.174, -0.127, -0.309 };

  // // RL
  // vec p3 = { -0.226, 0.128, -0.306 };

  // // RR
  // vec p4 = { -0.232, -0.127, -0.308 };

  // mat skew_p1 = skew_symmetric(p1);
  // mat skew_p2 = skew_symmetric(p2);
  // mat skew_p3 = skew_symmetric(p3);
  // mat skew_p4 = skew_symmetric(p4);

  // mat A(6, 12, arma::fill::zeros);
  // A.submat(0, 0, 2, 2) = eye(3, 3);
  // A.submat(0, 3, 2, 5) = eye(3, 3);
  // A.submat(0, 6, 2, 8) = eye(3, 3);
  // A.submat(0, 9, 2, 11) = eye(3, 3);

  // A.submat(3, 0, 5, 2) = skew_p1;
  // A.submat(3, 3, 5, 5) = skew_p2;
  // A.submat(3, 6, 5, 8) = skew_p3;
  // A.submat(3, 9, 5, 11) = skew_p4;

  // // A.print("A");

  // vec b(6);
  // b.rows(0, 2) = m * (pddotb + g);
  // b.rows(3, 5) = Ig * wdotb;
  // // b.print("b");

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
