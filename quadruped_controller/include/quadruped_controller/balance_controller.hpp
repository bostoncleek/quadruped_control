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

using rigid3d::Quaternion;
using rigid3d::Rotation3d;
using rigid3d::skew_symmetric;

using qpOASES::Options;
using qpOASES::QProblemB;
using qpOASES::real_t;
using qpOASES::returnValue;
using qpOASES::SQProblem;

static const uint64_t NUM_VARIABLES_QP = 12;
static const uint64_t NUM_CONSTRAINTS_QP = 20;
static const uint64_t NUM_EQUATIONS_QP = 6;

// static const uint64_t NUM_CONTACT_POINTS = 4;
// static const uint64_t NUM_VARIABLES_PER_FOOT = 3;
// static const uint64_t NUM_CONSTRAINTS_PER_FOOT = 5;


void copy_to_real_t(const vec& source, real_t* target);

void copy_to_real_t(const mat& source, real_t* target);

void print_real_t(const real_t* const array, unsigned int n_rows, unsigned int n_cols,
                  const std::string& msg = "");


class BalanceController
{
public:
  BalanceController(double mu, double mass, double fzmin, double fzmax, const mat& Ib,
                    const mat& S, const vec& kp_p, const vec& kd_p, const vec& kp_w,
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
  vec g_;        // gravity vector in world frame (x3)

  // PD control gains
  vec kp_p_;  // kp gain on COM position (x3)
  vec kd_p_;  // kd gain on COM linear velocity (x3)
  vec kp_w_;  // kp gain on COM orientaion (x3)
  vec kd_w_;  // kd gain on COM angular velocities (x3)

  // QP variables
  // QProblemB QPSolver_;
  SQProblem QPSolver_;

  double fzmin_, fzmax_;  // min and max normal reaction force (Newtons)
  bool qp_initialized_;   // QP initialized
  mat S_;                 // positive-definite weight on dynamics (6x6)
  mat C_;                 // constraints

  real_t qp_Q_[NUM_VARIABLES_QP * NUM_VARIABLES_QP];
  real_t qp_c_[NUM_VARIABLES_QP];

  real_t qp_C_[NUM_CONSTRAINTS_QP * NUM_VARIABLES_QP];
  real_t qp_lbC_[NUM_CONSTRAINTS_QP];
  real_t qp_ubC_[NUM_CONSTRAINTS_QP];

  real_t qp_lb_[NUM_VARIABLES_QP];
  real_t qp_ub_[NUM_VARIABLES_QP];
};

}  // namespace quadruped_controller

#endif
