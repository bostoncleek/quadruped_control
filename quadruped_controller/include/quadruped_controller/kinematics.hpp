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
#include <string>
#include <functional>
#include <cmath>

// Linear Algebra
#include <armadillo>

namespace quadruped_controller
{
using arma::mat;
using arma::mat33;
using arma::vec;
using arma::vec3;

/** @brief Kinematic model of a quarduped robot */
class QuadrupedKinematics
{
public:
  /** @brief Constructor */
  QuadrupedKinematics();

  /**
   * @brief Compose the position of all feet in body frame
   * @param q - joint angles for four legs (12x1)
   * @return postion of four feet relative to base_link (3x4)
   * @details The input vector, q, contains the joint angles of all four legs
   * [RL, FL, RR, FR] where each leg contains [hip, thigh, calf] joints. Each column
   * in the output corresponds to a foot position [x, y, z].
   */
  mat forwardKinematics(const vec& q) const;

  /**
   * @brief Compose the position of a single foot in body frame
   * @param q - joint angles [hip, thigh, calf] [x, y, z]
   * @return postion of four feet relative to base_link
   */
  vec3 forwardKinematics(const std::string& leg_name, const vec3& q) const;

  /**
   * @brief Inverse kinematics for a single leg
   * @param leg_name - name of leg
   * @param trans_bh - translation from base_link to hip link
   * @param links - leg link configuration [l1, l2, l3]
   * @param foothold - position of foot relative to base_link [x, y, z]
   * @return leg joint angle [hip, thigh, calf]
   */
  vec3 legInverseKinematics(const std::string& leg_name, const vec3& foothold) const;

  /**
   * @brief Compose the geometric jacobian for a single leg
   * @param leg_name - name of leg
   * @param joints - leg joint angles [hip, thigh, calf]
   * @return single leg jacobian (3x3)
   */
  mat33 legJacobian(const std::string& leg_name, const vec3& q) const;

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
  std::map<std::string, std::pair<vec3, vec3>> link_map_;
  vec3 links_;  // lengths [l1 l2 l3]
};
}  // namespace quadruped_controller
#endif
