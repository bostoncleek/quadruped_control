/**
 * @file types.hpp
 * @date 2021-04-3
 * @author Boston Cleek
 * @brief typdefs
 */

#ifndef TYPES_HPP
#define TYPES_HPP

// C++
#include <map>
#include <utility>
#include <string>

// Linear Algebra
#include <armadillo>

namespace quadruped_controller
{
using arma::mat;
using arma::mat33;
using arma::vec;
using arma::vec3;

/** @brief CoM state */
struct RobotStateCoM
{
  vec3 x;     // COM position in world [x, y, z]
  vec3 xdot;  // COM linear velocity in world [vx, vy, vz]
  vec3 w;     // COM angular velocity in world [wx, wy, wz]
  mat33 Rwb;  // rotation from world to COM (3x3)
};

/** @brief Leg state in gait */
enum LegState
{
  swing = 0,
  stance = 1
};

/** @brief map leg name to LegState and phase */
typedef std::map<std::string, std::pair<LegState, double>> GaitMap;

/** @brief map leg name to desired foot placement in world frame */
typedef std::map<std::string, vec3> FootholdMap;
}  // namespace quadruped_controller
#endif