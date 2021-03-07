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
                  const std::string& msg)
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


BalanceController::BalanceController(double mu, double mass, double fzmin, double fzmax,
                                     const mat& Ib, const mat& S, const vec& kp_p,
                                     const vec& kd_p, const vec& kp_w, const vec& kd_w)
  : mu_(mu)
  , mass_(mass)
  , Ib_(Ib)
  , g_({ 0.0, 0.0, -9.81 })
  , kp_p_(kp_p)
  , kd_p_(kd_p)
  , kp_w_(kp_w)
  , kd_w_(kd_w)
  , QPSolver_(NUM_VARIABLES_QP, NUM_CONSTRAINTS_QP)
  , fzmin_(fzmin)
  , fzmax_(fzmax)
  , qp_initialized_(false)
  , S_(S)
  , C_(NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP, arma::fill::zeros)
{
  initConstraints();
}


vec BalanceController::control(const mat& ft_p, const mat& Rwb, const mat& Rwb_d,
                               const vec& x, const vec& xdot, const vec& w,
                               const vec& x_d, const vec& xdot_d, const vec& w_d)
{
  // IMPORTANT: Ground reaction forces are in world frame
  // PD control on COM position and orientation
  // [R1] Eq(3)
  const vec xddot_d = kp_p_ % (x_d - x) + kd_p_ % (xdot_d - xdot);
  // xddot_d.print("xddot_d");

  // [R2] Proposition 2.5 and [R1] Eq(4)
  const Rotation3d R_error(Rwb_d * Rwb.t());
  const vec wdot_d = kp_w_ % R_error.angleAxisTotal() + kd_w_ % (w_d - w);
  // wdot_d.print("wdot_d");

  // [R1] Eq(5) Linear Euler's single rigid boyd dynamics
  const auto srb_dyn = dynamics(ft_p, Rwb, x, xddot_d, wdot_d);
  const mat A_dyn = std::get<mat>(srb_dyn);
  const vec b_dyn = std::get<vec>(srb_dyn);

  // [R1] Convert Eq(6) to QP standard form 1/2*x.T*Q*x + x.T*c
  // Q = 2*A.T*S*A (12x12)
  // c = -2*A.T*S*b (12x1)
  const mat Q = 2.0 * A_dyn.t() * S_ * A_dyn;
  const vec c = -2.0 * A_dyn.t() * S_ * b_dyn;
  // H.print("H");
  // g.print("g");

  copy_to_real_t(Q, qp_Q_);
  copy_to_real_t(c, qp_c_);
  // print_real_t(qp_H_, NUM_VARIABLES_QP, NUM_VARIABLES_QP);
  // print_real_t(qp_g_, NUM_VARIABLES_QP, 1);

  int nWSR = 100;
  real_t* qp_lb = nullptr;
  real_t* qp_ub = nullptr;

  // See Init Homotopy pg 31
  const returnValue retVal =
      QPSolver_.init(qp_Q_, qp_c_, qp_C_, qp_lb, qp_ub, qp_lbC_, qp_ubC_, nWSR);

  real_t xOpt[NUM_VARIABLES_QP];
  QPSolver_.getPrimalSolution(xOpt);

  print_real_t(xOpt, NUM_VARIABLES_QP, 1, "xOpt");

  // if (!qp_initialized_)
  // {
  //   qp_initialized_ = true;
  // }

  // else
  // {
  //   // use hotstart
  // }


  // Ground reaction forces in body frame
  vec fb(NUM_VARIABLES_QP, arma::fill::zeros);
  return fb;
}


tuple<mat, vec> BalanceController::dynamics(const mat& ft_p, const mat& Rwb, const vec& x,
                                            const vec& xddot_d, const vec& wdot_d) const
{
  // Vector from COM to each foot position
  const mat x_ft_p = ft_p.each_col() - x;

  // Moment of Interia in world frame
  const mat Iw = Rwb * Ib_ * Rwb.t();
  // Iw.print("Iw");

  mat A(NUM_EQUATIONS_QP, NUM_VARIABLES_QP, arma::fill::zeros);
  A.submat(0, 0, 2, 2) = eye(3, 3);
  A.submat(0, 3, 2, 5) = eye(3, 3);
  A.submat(0, 6, 2, 8) = eye(3, 3);
  A.submat(0, 9, 2, 11) = eye(3, 3);

  A.submat(3, 0, 5, 2) = skew_symmetric(x_ft_p.col(0));
  A.submat(3, 3, 5, 5) = skew_symmetric(x_ft_p.col(1));
  A.submat(3, 6, 5, 8) = skew_symmetric(x_ft_p.col(2));
  A.submat(3, 9, 5, 11) = skew_symmetric(x_ft_p.col(3));

  // A.print("Euler RBD: A");

  vec b(NUM_EQUATIONS_QP, arma::fill::zeros);
  b.rows(0, 2) = mass_ * (xddot_d + g_);
  b.rows(3, 5) = Iw * wdot_d;

  // b.print("Euler RBD: b");

  return std::make_tuple(A, b);
}

void BalanceController::initConstraints()
{
  const auto upper = 1000000.0;
  const auto lower = -1000000.0;

  // [R1] Eq(7) and Eq(8)
  // Constraint per foot
  const mat Ci = { { 1.0, 0.0, -mu_ },
                   { 0.0, 1.0, -mu_ },
                   { 0.0, 1.0, mu_ },
                   { 1.0, 0.0, mu_ },
                   { 0.0, 0.0, 1.0 } };

  // Lower and upper limits per foot
  const vec dli = { lower, lower, 0.0, 0.0, fzmin_ };
  const vec dui = { 0.0, 0.0, upper, upper, fzmax_ };

  // TODO: This will need to change when some feet
  //       are not in constact with the ground.
  C_.submat(0, 0, 4, 2) = Ci;
  C_.submat(5, 3, 9, 5) = Ci;
  C_.submat(10, 6, 14, 8) = Ci;
  C_.submat(15, 9, 19, 11) = Ci;

  vec lbC(NUM_CONSTRAINTS_QP);
  vec ubC(NUM_CONSTRAINTS_QP);

  lbC.rows(0, 4) = dli;
  lbC.rows(5, 9) = dli;
  lbC.rows(10, 14) = dli;
  lbC.rows(15, 19) = dli;

  ubC.rows(0, 4) = dui;
  ubC.rows(5, 9) = dui;
  ubC.rows(10, 14) = dui;
  ubC.rows(15, 19) = dui;

  copy_to_real_t(C_, qp_C_);
  copy_to_real_t(lbC, qp_lbC_);
  copy_to_real_t(ubC, qp_ubC_);

  // C_.print("C");
  // lbC.print("lbC");
  // ubC.print("ubC");

  // print_real_t(qp_C_, NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
  // print_real_t(qp_lbC_, NUM_CONSTRAINTS_QP, 1);
  // print_real_t(qp_ubC_, NUM_CONSTRAINTS_QP, 1);
}


}  // namespace quadruped_controller
