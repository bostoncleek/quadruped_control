/**
 * @file balance_controller.hpp
 * @date 2021-03-04
 * @author Boston Cleek
 * @brief Force balance controller
 */
#ifndef BALANCE_CONTROLLER_HPP
#define BALANCE_CONTROLLER_HPP

// QP solver
#include <qpOASES.hpp>

#include <quadruped_controller/math/rigid3d.hpp>

namespace quadruped_controller
{
using std::tuple;

using arma::eye;
using arma::mat;
using arma::vec;

using math::Quaternion;
using math::Rotation3d;
using math::skew_symmetric;

using qpOASES::Options;
using qpOASES::real_t;
using qpOASES::returnValue;
using qpOASES::SQProblem;


void copy_to_real_t(const vec& source, real_t* target);

void copy_to_real_t(const mat& source, real_t* target);

vec copy_from_real_t(const real_t* const source, unsigned int n_rows);

void print_real_t(const real_t* const array, unsigned int n_rows, unsigned int n_cols,
                  const std::string& msg = "");


class BalanceController
{
public:
  BalanceController(double mu, double mass, double fzmin, double fzmax, const mat& Ib,
                    const mat& S, const mat& W, const vec& kff, const vec& kp_p, const vec& kd_p, const vec& kp_w,
                    const vec& kd_w);


  vec control(const mat& ft_p, const mat& Rwb, const mat& Rwb_d, const vec& x,
              const vec& xdot, const vec& w, const vec& x_d, const vec& xdot_d,
              const vec& w_d);


private:
  tuple<mat, vec> dynamics(const mat& ft_p, const mat& Rwb, const vec& x,
                           const vec& xddot_d, const vec& wdot_d) const;

  void initConstraints();


private:
  // Dynamic properties
  double mu_;    // coefficient of friction
  double mass_;  // total mass of robot (kg)
  mat Ib_;       // moment of interia in body frame (3x3)
  const vec g_;  // gravity vector in world frame (x3)

  // PD control gains
  vec kff_;   // feed forward gains (x6)
  vec kp_p_;  // kp gain on COM position (x3)
  vec kd_p_;  // kd gain on COM linear velocity (x3)
  vec kp_w_;  // kp gain on COM orientaion (x3)
  vec kd_w_;  // kd gain on COM angular velocities (x3)

  // QP variables
  static const uint64_t num_equations_qp_{ 6 };
  static const uint64_t num_variables_qp_{ 12 };
  static const uint64_t num_constraints_qp_{ 20 };

  // QProblemB QPSolver_;
  SQProblem QPSolver_;

  const int nWSR_;        // max working set recalculations
  double fzmin_, fzmax_;  // min and max normal reaction force (Newtons)
  mat S_;                 // positive-definite weights on dynamics (6x6)
  mat W_;                 // positive-definite weights on GRFs (12x12)
  mat C_;                 // constraints

  const real_t cpu_time_;  // max CPU time for QP solution (s)
  real_t qp_Q_[num_variables_qp_ * num_variables_qp_];
  real_t qp_c_[num_variables_qp_];

  real_t qp_C_[num_constraints_qp_ * num_variables_qp_];
  real_t qp_lbC_[num_constraints_qp_];
  real_t qp_ubC_[num_constraints_qp_];

  // real_t qp_lb_[num_variables_qp_];
  // real_t qp_ub_[num_variables_qp_];
};
}  // namespace quadruped_controller
#endif
