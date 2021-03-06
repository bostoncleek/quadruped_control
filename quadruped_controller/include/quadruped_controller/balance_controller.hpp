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
using qpOASES::QProblem;
using qpOASES::QProblemB;
using qpOASES::real_t;
using qpOASES::returnValue;

static const uint64_t NUM_VARIABLES_QP = 12;


class BalanceController
{
public:
  BalanceController(double mass, const mat& Ib, const vec& kp_p, const vec& kd_p,
                    const vec& kp_w, const vec& kd_w);


  vec control(const mat& ft_p, const mat& Rb, const mat& Rb_d, const vec& x,
              const vec& xdot, const vec& w, const vec& x_d, const vec& xdot_d,
              const vec& w_d);


private:
  tuple<mat, vec> constructDynamics(const mat& ft_p, const mat& Rb, const vec& x,
                                    const vec& xddot_d, const vec& wdot_d) const;


private:
  double mass_;  // Total mass of robot (kg)

  mat Ib_;  // Moment of interia in body frame

  vec kp_p_;  // kp gain on COM position (x3)
  vec kd_p_;  // kd gain on COM linear velocity (x3)
  vec kp_w_;  // kp gain on COM orientaion (x3)
  vec kd_w_;  // kd gain on COM angular velocities (x3)

  vec g_;  // Gravity vector in world frame (x3)

  // QP variables
  real_t qp_Q_[NUM_VARIABLES_QP * NUM_VARIABLES_QP];
  real_t qp_c_[NUM_VARIABLES_QP];
};

}  // namespace quadruped_controller

#endif
