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
  FootTrajectory();

  bool generateTrajetory(const vec3& p_start, const vec3& p_center, const vec3& p_final) const;

  FootState trackTrajectory(double t) const;  

private:
  mat initSystem() const;

  mat constructConstraints(const vec3& p_start, const vec3& p_center, const vec3& p_final) const;

private:  
  mat A_; // system  
  mutable mat coefficients_; // polynomial coefficients
};


}  // namespace quadruped_controller
#endif
