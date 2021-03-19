/**
 * @file trajectory.cpp
 * @date 2021-03-12
 * @author Boston Cleek
 * @brief Base and leg reference trajectory generators
 */

// Quadruped Control
#include <quadruped_controller/trajectory.hpp>
#include <quadruped_controller/math/numerics.hpp>

namespace quadruped_controller
{
using std::cos;
using std::sin;

using math::Transform3d;

Pose integrate_twist_yaw(const Pose& pose, const vec& u, double dt)
{
  // change in angle axis
  const vec3 delta_aa = u.rows(3, 5) * dt;
  const double angle = arma::norm(delta_aa);

  // rotation from b to b'
  mat Rbbp = arma::eye(3, 3);
  // translation from b to b'
  vec3 tbbp(arma::fill::zeros);

  // no rotation
  if (math::almost_equal(angle, 0.0))
  {
    tbbp = u.rows(0, 2) * dt;
  }

  else
  {
    // construct rotation from change in anle axis
    const vec3 axis = delta_aa / angle;
    Rbbp = Quaternion(angle, axis).matrix();

    // rotate translation
    tbbp = Rbbp * u.rows(0, 2) * dt;
  }

  // Extract yaw
  const vec3 euler_angles = pose.transform().getQuaternion().eulerAngles();

  // Twb' based on yaw only
  const Rotation3d Rwb_yaw(0.0, 0.0, euler_angles(2));
  const Transform3d Twb_yaw(Rwb_yaw, pose.position);

  // Twb' = Twb * Tbb'
  // const Transform3d Twbp = pose.transform() * Transform3d(Rbbp, tbbp);
  const Transform3d Twbp = Twb_yaw * Transform3d(Rbbp, tbbp);

  return Pose(Twbp);
}

/////////////////////////////////////////////////////////
// StanceBaseControl
StanceBaseControl::StanceBaseControl()
{
}

StanceBaseControl::StanceBaseControl(const Pose& pose) : pose_(pose)
{
}

void StanceBaseControl::setPose(const Pose& pose)
{
  pose_ = pose;
}

Pose StanceBaseControl::integrateTwist(const Pose& pose, const vec& u, double dt)
{
  // TODO update when error becomes too large
  if (i == update)
  {
    pose_ = pose;
  }

  i++;

  // change in angle axis
  const vec3 delta_aa = u.rows(3, 5) * dt;
  const double angle = arma::norm(delta_aa);

  // rotation from b to b'
  mat Rbbp = arma::eye(3, 3);
  // translation from b to b'
  vec3 tbbp(arma::fill::zeros);

  // no rotation
  if (math::almost_equal(angle, 0.0))
  {
    tbbp = u.rows(0, 2) * dt;
  }

  else
  {
    // construct rotation from change in anle axis
    const vec3 axis = delta_aa / angle;
    Rbbp = Quaternion(angle, axis).matrix();

    // rotate translation
    tbbp = Rbbp * u.rows(0, 2) * dt;
  }

  // Twb' = Twb * Tbb'
  const Transform3d Twbp = pose_.transform() * Transform3d(Rbbp, tbbp);

  // Update internal pose
  pose_ = Pose(Twbp);
  return pose_;
}

/////////////////////////////////////////////////////////
// BaseTrajectory
BaseTrajectory::BaseTrajectory()
{
}
}  // namespace quadruped_controller
