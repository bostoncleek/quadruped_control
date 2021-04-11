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

/** @brief Foot state in cartesian space */
struct FootState
{
  FootState() = default;

  FootState(const vec3& position, const vec3& velocity)
    : position(position), velocity(velocity)
  {
  }

  vec3 position;  // position [x, y, z]
  vec3 velocity;  // linear velocity [vx, vy, vz]
};

/** @brief Foot trajectory position boundary conditions */
struct FootTrajBounds
{
  FootTrajBounds() = default;

  FootTrajBounds(const vec3& p_start, const vec3& p_final)
    : p_start(p_start), p_final(p_final)
  {
  }

  vec3 p_start;  // initial position [x, y, z]
  vec3 p_final;  // final position [x, y, z]
};

/** @brief Scheduled phases for leg*/
struct LegScheduledPhases
{
  double stance_start;
  double stance_end;
  double swing_start;
  double swing_end;
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

/** @brief map leg name to foot position and velocity in world frame */
typedef std::map<std::string, FootState> FootStateMap;

/** @brief map leg name to foot trajectory boundary conditions */
typedef std::map<std::string, FootTrajBounds> FootTrajBoundsMap;

/** @brief map leg name to scheduled phases */
typedef std::map<std::string, LegScheduledPhases> ScheduledPhasesMap;

}  // namespace quadruped_controller
#endif
