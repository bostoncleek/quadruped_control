/**
 * @file trajectory.cpp
 * @date 2021-03-12
 * @author Boston Cleek
 * @brief Base and leg reference trajectory generators
 */

// C++ 
#include <exception>

// Quadruped Control
#include <quadruped_controller/trajectory.hpp>
#include <quadruped_controller/math/numerics.hpp>

namespace quadruped_controller
{
using std::cos;
using std::sin;
using std::pow;

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
// BaseTrajectory::BaseTrajectory()
// {
// }

/////////////////////////////////////////////////////////
// FootTrajectory
FootTrajectory::FootTrajectory() : A_(initSystem())
{
}

bool FootTrajectory::generateTrajetory(const vec3& p_start, const vec3& p_center, const vec3& p_final) const
{
  const mat B = constructConstraints(p_start, p_center, p_final);
  return arma::solve(coefficients_, A_, B, arma::solve_opts::fast);
}

FootState FootTrajectory::trackTrajectory(double t) const
{
  if (t < 0.0 || t > 1.0)
  {
    throw std::invalid_argument("t must be on domain [0 1]");
  }

  const vec position_factors = 
    { 1.0, t, pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5), pow(t, 6) };

  const vec velocity_factors = 
    { 0.0, 1.0, 2.0 * t, 3.0 * pow(t, 2), 4.0 * pow(t, 3), 5.0 * pow(t, 4), 6.0 * pow(t, 5) };

  const vec3 p_ref = (position_factors.t() * coefficients_).t();
  const vec3 v_ref = (velocity_factors.t() * coefficients_).t();

  return FootState(p_ref, v_ref);
}

mat FootTrajectory::initSystem() const
{
  // t:[0, 1]
  // position: s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^6
  // linear velocity; sdot(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4 + 6*a6*t^5
  // linear acceleration sddot(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3 + 30*a6*t^4
  // constraints:
  // positions (start, final, center) s(0) = p0, s(1) = pf, s(0.5) = pc 
  // linear velocities: sdot(0) = sdot(1) = 0
  // linear accelerations sddot(0) = sddot(1) = 0

  const mat A = { {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                  {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
                  {1.0, 0.5, pow(0.5, 2), pow(0.5, 3), pow(0.5, 4), pow(0.5, 5), pow(0.5, 6)},
                  {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                  {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
                  {0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0},
                  {0.0, 0.0, 2.0, 6.0, 12.0, 20.0, 30.0} };

  return A;
}

mat FootTrajectory::constructConstraints(const vec3& p_start, const vec3& p_center, const vec3& p_final) const
{
  // Trajectory constraints
  // [x0  y0  z0]
  // [xf  yf  zf]
  // [xc  yc  zc]
  // [vx0 vy0 vz0]
  // [vxf vyf vzf]
  // [ax0 ay0 az0]
  // [axf ayf azf]
  mat B(7, 3, arma::fill::zeros);
  B.row(0) = p_start.t(); 
  B.row(1) = p_final.t();
  B.row(2) = p_center.t();

  return B;
}

}  // namespace quadruped_controller
