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
#include <quadruped_controller/gait.hpp>

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

/**
 * @brief Copy vector to array
 * @param source - vector
 * @param target[out] - array containing same contents as the source
 */
void copy_to_real_t(const vec& source, real_t* target);

/**
 * @brief Copy matrix to array
 * @param source - matrix
 * @param target[out] - array containing same contents as the source
 */
void copy_to_real_t(const mat& source, real_t* target);

/**
 * @brief Copy array to vector
 * @param source - array
 * @param n_rows - number of rows in output vector
 * @return vector containing same contents as source
 */
vec copy_from_real_t(const real_t* const source, unsigned int n_rows);

/**
 * @brief Print array contents to stdout
 * @param array - array to print
 * @param n_rows - number of rows
 * @param n_cols - number of columns
 * @param msg - optional print message
 */
void print_real_t(const real_t* const array, unsigned int n_rows, unsigned int n_cols,
                  const std::string& msg = "");

/** @brief Reactive optimal control strategy */
class BalanceController
{
public:
  /**
   * @brief Constructor
   * @param mu - friction coefficient (kg*m/s^2)
   * @param mass - total mass (kg)
   * @param fzmin - minimum z-axis ground reaction force (N)
   * @param fzmax - maximum z-axis ground reaction force (N)
   * @param Ib - trunk moment of inertia (kg*m^2)
   * @param S - positive-definite weight matrix on least sqaures (6x6)
   * @param W - positive-definite weight matrix on GRFs (12x12)
   * @param kff - COM feedforward gains (6x1)
   * @param kp_p - kp gain on COM position (3x1)
   * @param kd_p - kd gain on COM linear velocity (3x1)
   * @param kp_w - kp gain on COM orientaion (3x1)
   * @param kd_w - kd gain on COM angular velocities (3x1)
   * @param leg_names - vector of legs names
   */
  BalanceController(double mu, double mass, double fzmin, double fzmax, const mat& Ib,
                    const mat& S, const mat& W, const vec& kff, const vec& kp_p,
                    const vec& kd_p, const vec& kp_w, const vec& kd_w,
                    const std::vector<std::string>& leg_names);

  /**
   * @brief Compose ground reaction forces
   * @param ft_p - postions of feet in body frame (COM) (3x4)
   * @param Rwb - rotation from world to base_link (3x3)
   * @param Rwb_d - desired rotation from world to base_link (3x3)
   * @param x - COM position in world [x, y, z] (3x1)
   * @param xdot - COM linear velocity in world [vx, vy, vz] (3x1)
   * @param w - COM angular velocity in world [wx, wy, wz] (3x1)
   * @param x_d - desired COM position in world [x, y, z] (3x1)
   * @param xdot_d - desired COM linear velocity in world [vx, vy, vz] (3x1)
   * @param w_d - desired COM angular velocity in world [wx, wy, wz] (3x1)
   * @param gait_map - gait schedule
   * @return ground reaction forces in body frame (12x1)
   */
  vec control(const mat& ft_p, const mat& Rwb, const mat& Rwb_d, const vec& x,
              const vec& xdot, const vec& w, const vec& x_d, const vec& xdot_d,
              const vec& w_d, const GaitMap& gait_map = make_stance_gait()) const;

private:
  /**
   * @brief Compose linear Newton-Euler single rigid body dynamics
   * @param ft_p - postions of feet body frame (COM)  (3x4)
   * @param Rwb - rotation from world to base_link (3x3)
   * @param x - COM position in world [x, y, z] (3x1)
   * @param xddot_d - desired COM linear acceleration (3x1)
   * @param wdot_d - desired COM angular acceleration (3x1)
   * @return Newton-Euler dynamics written as a linear problem Ax = b [R1] Eq(5)
   * @details Euler single rigid body dyanmics are described by tau = I*wdot + w x (I*w).
   * The cross product term (w x (I*w)) is assumed to be ~ 0, therefore, the dynamics are
   * linear. The cross product term is small for bodies with small angular velocities.
   */
  tuple<mat, vec> dynamics(const mat& ft_p, const mat& Rwb, const vec& x,
                           const vec& xddot_d, const vec& wdot_d) const;

private:
  /** 
  * @brief Construct friction cone contraint
  * @return friction cone constraint matrix (20x12)
  * @details The matrix diagonal contains the friction cone for each 
  * leg and all other elements are zero. 
  */
  mat frictionConeConstraint() const;

  /** 
  * @brief Set friction code constraint lower and upper bounds
  * @param gait_map - gait schedule
  * @details If a foot is in swing phase the constraint bounds lower = upper = 0,
  * resulting in a zero vector ground reaction force.
  */
  void frictionConeBounds(const GaitMap& gait_map) const;

private:
  // Dynamic properties
  double mu_;    // coefficient of friction (kg*m/s^2)
  double mass_;  // total mass of robot (kg)
  mat Ib_;       // moment of interia in body frame (kg*m^2) (3x3)
  vec g_;        // gravity vector in world frame (m/s^2) (3x1)

  // PD control gains
  vec kff_;   // feed forward gains (6x1)
  vec kp_p_;  // kp gain on COM position (3x1)
  vec kd_p_;  // kd gain on COM linear velocity (3x1)
  vec kp_w_;  // kp gain on COM orientaion (3x1)
  vec kd_w_;  // kd gain on COM angular velocities (3x1)

  // QP variables
  static const uint64_t num_equations_qp_{ 6 };     // number of equations
  static const uint64_t num_variables_qp_{ 12 };    // number of variable (GRFs)
  static const uint64_t num_constraints_qp_{ 20 };  // total constraints (5 per foot)

  mutable SQProblem QPSolver_;  // sequential QP solver

  int nWSR_;              // max working set recalculations
  double fzmin_, fzmax_;  // min and max normal reaction force (N)
  mat S_;                 // positive-definite weight matrix on least sqaures (6x6)
  mat W_;                 // positive-definite weight matrix on GRFs (12x12)
  mat C_;                 // friction cone constraint matrix (20x12)

  real_t cpu_time_;  // max CPU time for QP solution (s)
  // QP standard form 1/2*x.T*Q*x + x.T*c
  mutable real_t qp_Q_[num_variables_qp_ * num_variables_qp_];
  mutable real_t qp_c_[num_variables_qp_];

  mutable real_t qp_C_[num_constraints_qp_ * num_variables_qp_];  // constraint matrix
  mutable real_t qp_lbC_[num_constraints_qp_];  // constraint lower bounds
  mutable real_t qp_ubC_[num_constraints_qp_];  // constraint upper bounds

  // Robot configuration
  std::vector<std::string> leg_names_;
};
}  // namespace quadruped_controller
#endif
