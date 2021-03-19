/**
 * @file kinematics.hpp
 * @date 2021-09-03
 * @author Boston Cleek
 * @brief Quadruped kinematics
 */
#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

// C++
#include <map>
#include <vector>
#include <string>
#include <functional>
#include <cmath>

// Linear Algebra
#include <armadillo>

namespace quadruped_controller
{
using arma::mat;
using arma::vec;

using std::map;
using std::string;
using std::vector;

using std::cos;
using std::sin;

/**
 * @brief Compose the geometric jacobian for a single leg
 * @param links - leg link configuration [l1, l2, l3]
 * @param joints - leg joint angles [hip, thigh, calf]
 * @return single leg jacobian (3x3)
 */
mat leg_jacobian(const vec& links, const vec& joints);

/**
 * @brief Forward kinematics for a single leg
 * @param trans_bh - translation from base_link to hip link
 * @param links - leg link configuration [l1, l2, l3]
 * @param joints - leg joint angles [hip, thigh, calf]
 * @return position of foot [x, y, z]
 */
vec leg_forward_kinematics(const vec& trans_bh, const vec& links, const vec& joints);

/** @brief Kinematic model of a quarduped robot */
class QuadrupedKinematics
{
public:
  /** @brief Constructor */
  QuadrupedKinematics();

  /**
   * @brief Compose the position of all feet
   * @param q - joint angles for four legs (12x1)
   * @return postion of four feet (3x4)
   * @details The input vector, q, contains the joint angles of all four legs
   * [RL, FL, RR, FR] where each leg contains [hip, thigh, calf] joints. Each column
   * in the output corresponds to a foot position [x, y, z].
   */
  mat forwardKinematics(const vec& q) const;

  /**
   * @brief Compose the joint torques for four legs based on the force applied to the foot
   * @param q - joint angles for four legs (12x1)
   * @param f - force applied to all four feet (12x1)
   * @return joint torques to achieve the desired force applied to foot
   * @details The forces correspond to all four legs [RL, FL, RR, FR] where
   * each force is of the format [fx, fy, fz] in the body frame.
   */
  vec jacobianTransposeControl(const vec& q, const vec& f) const;

private:
  // Map leg name to leg link configuration and translation from base to hip
  map<string, std::pair<vec, vec>> link_map_;
};
}  // namespace quadruped_controller
#endif
