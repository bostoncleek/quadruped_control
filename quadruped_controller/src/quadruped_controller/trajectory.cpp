/**
 * @file trajectory.cpp
 * @date 2021-03-12
 * @author Boston Cleek
 * @brief Base and leg reference trajectory generators
 */

// C++
#include <exception>

// ROS
#include <ros/console.h>

// Quadruped Control
#include <quadruped_controller/trajectory.hpp>
#include <quadruped_controller/math/numerics.hpp>

namespace quadruped_controller
{
static const std::string LOGNAME = "Trajectory Generator";

using std::cos;
using std::pow;
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
// BaseTrajectory::BaseTrajectory()
// {
// }

/////////////////////////////////////////////////////////
// FootTrajectory
FootTrajectory::FootTrajectory()
  : A_(initSystem()), coefficients_(7, 3, arma::fill::zeros)
{
}

bool FootTrajectory::generateTrajetory(const vec3& p_start, const vec3& p_center,
                                       const vec3& p_final) const
{
  const mat B = constantTerms(p_start, p_center, p_final);
  return arma::solve(coefficients_, A_, B, arma::solve_opts::fast);
}

FootState FootTrajectory::trackTrajectory(double t) const
{
  if (t < 0.0 || t > 1.0)
  {
    throw std::invalid_argument("t must be on domain [0 1]");
  }

  // s(t)
  const vec position_factors = { 1.0,       t,         pow(t, 2), pow(t, 3),
                                 pow(t, 4), pow(t, 5), pow(t, 6) };

  // sdot(t)
  const vec velocity_factors = {
    0.0, 1.0, 2.0 * t, 3.0 * pow(t, 2), 4.0 * pow(t, 3), 5.0 * pow(t, 4), 6.0 * pow(t, 5)
  };

  const vec3 p_ref = (position_factors.t() * coefficients_).t();
  const vec3 v_ref = (velocity_factors.t() * coefficients_).t();

  // const vec accel_factors = {
  //   0.0, 0.0, 2.0, 6.0 * t, 12.0 * pow(t, 2), 20.0 * pow(t, 3), 30.0 * pow(t, 3)
  // };

  // const vec3 a_ref = (accel_factors.t() * coefficients_).t();
  // a_ref.print("a_ref");

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

  const mat A = { { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
                  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },
                  { 1.0, 0.5, pow(0.5, 2), pow(0.5, 3), pow(0.5, 4), pow(0.5, 5),
                    pow(0.5, 6) },
                  { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
                  { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 },
                  { 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0 },
                  { 0.0, 0.0, 2.0, 6.0, 12.0, 20.0, 30.0 } };

  return A;
}

mat FootTrajectory::constantTerms(const vec3& p_start, const vec3& p_center,
                                  const vec3& p_final) const
{
  // Trajectory constraints
  // [x0  y0  z0] = p0
  // [xf  yf  zf] = pf
  // [xc  yc  zc] = pc
  // [vx0 vy0 vz0] = 0
  // [vxf vyf vzf] = 0
  // [ax0 ay0 az0] = 0
  // [axf ayf azf] = 0
  mat B(7, 3, arma::fill::zeros);
  B.row(0) = p_start.t();
  B.row(1) = p_final.t();
  B.row(2) = p_center.t();

  return B;
}

/////////////////////////////////////////////////////////
// FootTrajectoryManager
FootTrajectoryManager::FootTrajectoryManager(double height, double t_swing,
                                             double t_stance)
  : height_(height)
  , stance_phase_(t_stance / (t_swing + t_stance))
  , slope_(1.0 / (1.0 - stance_phase_))
  , y_intercept_(1.0 - slope_)
{
  // traj_map_.emplace("RL", FootTrajectory());
  // traj_map_.emplace("FL", FootTrajectory());
  // traj_map_.emplace("RR", FootTrajectory());
  // traj_map_.emplace("FR", FootTrajectory());
}

FootStateMap
FootTrajectoryManager::referenceStates(const GaitMap& gait_map,
                                       const FootTrajBoundsMap& foot_traj_map) const
{
  // Reference foot positions and velocities
  FootStateMap foot_state_map;

  // clear previous trajectories
  traj_map_.clear();

  // Generate trajectorys for these legs
  for (const auto& [leg_name, foot_traj_bounds] : foot_traj_map)
  {
    // The max height of the foot is in the center of the trajectory
    vec3 p_center = (foot_traj_bounds.p_start + foot_traj_bounds.p_final) / 2.0;
    p_center(2) = height_;

    FootTrajectory foot_traj;
    if (!foot_traj.generateTrajetory(foot_traj_bounds.p_start, p_center,
                                     foot_traj_bounds.p_final))
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to generate foot trajectory for leg: %s",
                      leg_name.c_str());
    }
    else
    {
      traj_map_.emplace(leg_name, foot_traj);
    }

    // Get reference foot states for leg
    const FootState foot_state = referenceState(leg_name, gait_map.at(leg_name).second);
    foot_state_map.emplace(leg_name, foot_state);
  }

  return foot_state_map;
}

FootStateMap FootTrajectoryManager::referenceStates(const GaitMap& gait_map) const
{
  // Reference foot positions and velocities
  FootStateMap foot_state_map;

  for (const auto& [leg_name, leg_state] : gait_map)
  {
    if (leg_state.first == LegState::swing)
    {
      const auto search = traj_map_.find(leg_name);
      if (search != traj_map_.end())
      {
        // Get reference foot states for leg
        const FootState foot_state =
            referenceState(leg_name, gait_map.at(leg_name).second);
        foot_state_map.emplace(leg_name, foot_state);
      }
      else
      {
        ROS_ERROR_NAMED(LOGNAME,
                        "Failed to find trajectory for leg: %s. May need to re-plan foot "
                        "trajectories.",
                        leg_name.c_str());
      }
    }
  }

  return foot_state_map;
}

FootState FootTrajectoryManager::referenceState(const std::string& leg_name,
                                                double phase) const
{
  // Time in the trajecotry is a function of the swing phase i.e trajectory(t(phase))
  const auto t = slope_ * phase + y_intercept_;
  return traj_map_.at(leg_name).trackTrajectory(t);
}

}  // namespace quadruped_controller
