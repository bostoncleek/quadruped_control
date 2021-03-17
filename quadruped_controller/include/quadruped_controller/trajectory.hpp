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

// Linear Algebra
#include <armadillo>

// Quadruped Control
#include <quadruped_controller/math/rigid3d.hpp>

namespace quadruped_controller
{
using arma::mat;
using arma::vec;
using arma::vec3;

using quadruped_controller::math::Quaternion;
using quadruped_controller::math::Rotation3d;
using quadruped_controller::math::Transform3d;

struct Pose
{
  Pose() : position(arma::fill::zeros)
  {
  }

  Pose(const vec3& p) : position(p)
  {
  }

  Pose(const Quaternion& q) : orientation(q)
  {
  }

  Pose(const Transform3d& T)
    : orientation(T.getQuaternion()), position(T.getTranslation())
  {
  }

  Pose(const Quaternion& q, const vec3& p) : orientation(q), position(p)
  {
  }

  Pose(const Rotation3d& R, const vec3& p) : orientation(R), position(p)
  {
  }

  Pose(const mat& R, const vec3& p) : orientation(R), position(p)
  {
  }

  Transform3d transform() const
  {
    return Transform3d(orientation, position);
  }

  void print(const std::string& msg = "") const
  {
    if (!msg.empty())
    {
      std::cout << msg << "\n";
    }
    orientation.print("orientation: ");
    position.print("position: ");
  }

  Quaternion orientation;
  vec3 position;  // (x,y,z)
};


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


/** @brief Base control (z, roll, pitch, yaw) only when standing
 */
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
class BaseTrajectory
{
public:
  BaseTrajectory();

private:
};
}  // namespace quadruped_controller
#endif
