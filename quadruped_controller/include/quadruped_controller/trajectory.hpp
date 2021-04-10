/**
 * @file trajectory.hpp
 * @date 2021-03-12
 * @author Boston Cleek
 * @brief Base and leg reference trajectory generators
 */
#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

// C++
#include <cmath>

// Quadruped Control
#include <quadruped_controller/math/rigid3d.hpp>
#include <quadruped_controller/types.hpp>

namespace quadruped_controller
{
using arma::mat;
using arma::vec;
using arma::vec3;

using quadruped_controller::math::Pose;
using quadruped_controller::math::Quaternion;
using quadruped_controller::math::Rotation3d;
using quadruped_controller::math::Transform3d;

/**
 * @brief Integrate a constant twist
 * @param pose - position and orientaion of rigid body
 * @param vb - body twist [vx, vy, vz, wx, wy, wz]
 * @param dt - time step
 * @return new pose
 * @details Only considers [vx, vy, vz, wz] and sets the roll and pitch
 *  angles in the returned post to zero.
 */
Pose integrate_twist_yaw(const Pose& pose, const vec& u, double dt);

/** @brief Base control (z, roll, pitch, yaw) only when standing */
class StanceBaseControl
{
public:
  StanceBaseControl();

  StanceBaseControl(const Pose& pose);

  void setPose(const Pose& pose);

  Pose integrateTwist(const Pose& pose, const vec& u, double dt);

private:
  Pose pose_;
  const unsigned int update{ 5 };
  unsigned int i{ 0 };
};

/** @brief COM reference trajectory generator */
// class BaseTrajectory
// {
// public:
//   BaseTrajectory();

// private:
// };

/** @brief Generates a trajectory for a single foot */
class FootTrajectory
{
public:
  /** @brief Constructor */
  FootTrajectory();

  /**
   * @brief Generates trajectory
   * @param p_start - foot start position [x y z]
   * @param p_center - desired foot position at trajectory center [xc, yc, zc]
   * @param p_final - desired final foot position [xf yf zf]
   * @return true if the trajectory is successfully generated
   * @details The trajectory is generated using a sextic polynomial and consists
   * of the foot's position and velocity on the time domain [0 1]. The initial and
   * terminal velocities and accelerations are zero.
   */
  bool generateTrajetory(const vec3& p_start, const vec3& p_center,
                         const vec3& p_final) const;

  /**
   * @brief Retrieve the foot state at a given time
   * @param t - time [0 1]
   * @return desired position and velocity of foot
   */
  FootState trackTrajectory(double t) const;

private:
  /**
   * @brief Construct system of equations
   * @return system (7x7)
   * @details In a linear system AX = B, this constructs A
   */
  mat initSystem() const;

  /**
   * @brief Construct constant terms
   * @param p_start - foot start position [x y z]
   * @param p_center - desired foot position at trajectory center [xc, yc, zc]
   * @param p_final - desired final foot position [xf yf zf]
   * @return constant terms (7x3)
   * @details In a linear system AX = B, this constructs B
   */
  mat constantTerms(const vec3& p_start, const vec3& p_center, const vec3& p_final) const;

private:
  mat A_;                     // system (7x7)
  mutable mat coefficients_;  // polynomial coefficients (7x3)
};

/** @brief Manages foot trajectories */
class FootTrajectoryManager
{
public:
  FootTrajectoryManager(double height, double t_swing, double t_stance);

  FootStateMap referenceStates(const GaitMap& gait_map,
                               const FootTrajBoundsMap& foot_traj_map) const;

  FootStateMap referenceStates(const GaitMap& gait_map) const;

  FootState referenceState(const std::string& leg_name, double phase) const;

private:
  double height_;        // max height in foot trajectory
  double stance_phase_;  // when stance phase ends [0 1)
  double slope_;         // scale trajectory via interpolation
  double y_intercept_;   // scale trajectory via interpolation
  mutable std::map<std::string, FootTrajectory>
      traj_map_;  // map leg name to foot trajectory
};

}  // namespace quadruped_controller
#endif
