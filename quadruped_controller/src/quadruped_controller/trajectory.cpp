/**
 * @file trajectory.cpp
 * @date 2021-03-12
 * @author Boston Cleek
 * @brief Base and leg reference trajectory generators
 */

// C++
#include <exception>
#include <algorithm>

// ROS
#include <ros/console.h>

// Quadruped Control
#include <quadruped_controller/trajectory.hpp>
#include <quadruped_controller/math/numerics.hpp>

namespace quadruped_controller
{
static const std::string LOGNAME = "trajectory_generator";

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

  // Twb based on yaw only -> prevents robot from drifting in the roll
  // and pitch axises
  const Rotation3d Rwb_yaw(0.0, 0.0, euler_angles(2));
  const Transform3d Twb_yaw(Rwb_yaw, pose.position);

  // Twb' = Twb * Tbb'
  // const Transform3d Twbp = pose.transform() * Transform3d(Rbbp, tbbp);
  const Transform3d Twbp = Twb_yaw * Transform3d(Rbbp, tbbp);

  return Pose(Twbp);
}

/////////////////////////////////////////////////////////
// SupportPolygon
SupportPolygon::SupportPolygon()
{
  adjacent_leg_map_.emplace("RL", std::make_pair("FL", "RR"));
  adjacent_leg_map_.emplace("FL", std::make_pair("FR", "RL"));
  adjacent_leg_map_.emplace("FR", std::make_pair("RR", "FL"));
  adjacent_leg_map_.emplace("RR", std::make_pair("RL", "FR"));
}

vec SupportPolygon::position(const ScheduledPhasesMap& phase_map,
                             const FootholdMap& foot_map, const GaitMap& gait_map) const
{
  // map leg name -> total weight for foot
  std::map<std::string, double> weight_map;

  // constant and
  const auto root2 = std::sqrt(2.0);

  // Compose weights
  for (const auto& [leg_name, adjacent_legs] : adjacent_leg_map_)
  {
    const auto phase = gait_map.at(leg_name).second;

    auto weight = 0.0;
    if (gait_map.at(leg_name).first == LegState::stance)
    {
      const auto c0 = phase_map.at(leg_name).stance_start;
      const auto cf = phase_map.at(leg_name).stance_end;

      // add epsilon to prevent division by 0
      weight = 0.5 * (std::erf(phase / (c0 * root2 + 1.0e-12)) +
                      std::erf((1.0 - phase) / (cf * root2 + 1.0e-12)));
    }
    else
    {
      const auto s0 = phase_map.at(leg_name).swing_start;
      const auto sf = phase_map.at(leg_name).swing_end;

      // add epsilon to prevent division by 0
      weight = 0.5 * (2.0 + std::erf(-phase / (s0 * root2 + 1.0e-12)) +
                      std::erf((phase - 1.0) / (sf * root2 + 1.0e-12)));
    }

    weight_map.emplace(leg_name, weight);
  }

  mat supports(2, 4);
  unsigned int i = 0;
  // Compose virtual points for all legs
  for (const auto& [leg_name, weight] : weight_map)
  {
    // Only use x,y positions
    // foot position
    const vec p = foot_map.at(leg_name).rows(0, 1);
    // clockwise foot position
    const vec p_minus = foot_map.at(adjacent_leg_map_.at(leg_name).first).rows(0, 1);
    // counter clockwise foot position
    const vec p_plus = foot_map.at(adjacent_leg_map_.at(leg_name).second).rows(0, 1);

    const vec zeta_minus = p * weight + p_minus * (1.0 - weight);
    const vec zeta_plus = p * weight + p_plus * (1.0 - weight);

    const auto w_minus = weight_map.at(adjacent_leg_map_.at(leg_name).first);
    const auto w_plus = weight_map.at(adjacent_leg_map_.at(leg_name).second);

    supports.col(i) = (1.0 / (weight + w_minus + w_plus)) *
                      (weight * p + w_minus * zeta_minus + w_plus * zeta_plus);
    i++;
  }

  vec zeta(2);
  zeta.row(0) = arma::sum(supports.row(0)) / 4.0;
  zeta.row(1) = arma::sum(supports.row(1)) / 4.0;

  return zeta;
}

// ///////////////////////////////////////////////////////
// StanceBaseControl
// StanceBaseControl::StanceBaseControl()
// {
// }

// StanceBaseControl::StanceBaseControl(const Pose& pose) : pose_(pose)
// {
// }

// void StanceBaseControl::setPose(const Pose& pose)
// {
//   pose_ = pose;
// }

// Pose StanceBaseControl::integrateTwist(const Pose& pose, const vec& u, double dt)
// {
//   // TODO update when error becomes too large
//   if (i == update)
//   {
//     pose_ = pose;
//   }

//   i++;

//   // change in angle axis
//   const vec3 delta_aa = u.rows(3, 5) * dt;
//   const double angle = arma::norm(delta_aa);

//   // rotation from b to b'
//   mat Rbbp = arma::eye(3, 3);
//   // translation from b to b'
//   vec3 tbbp(arma::fill::zeros);

//   // no rotation
//   if (math::almost_equal(angle, 0.0))
//   {
//     tbbp = u.rows(0, 2) * dt;
//   }

//   else
//   {
//     // construct rotation from change in anle axis
//     const vec3 axis = delta_aa / angle;
//     Rbbp = Quaternion(angle, axis).matrix();

//     // rotate translation
//     tbbp = Rbbp * u.rows(0, 2) * dt;
//   }

//   // Twb' = Twb * Tbb'
//   const Transform3d Twbp = pose_.transform() * Transform3d(Rbbp, tbbp);

//   // Update internal pose
//   pose_ = Pose(Twbp);
//   return pose_;
// }

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
    if (foot_traj.generateTrajetory(foot_traj_bounds.p_start, p_center,
                                    foot_traj_bounds.p_final))
    {
      traj_map_.emplace(leg_name, foot_traj);
    }

    else
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to generate foot trajectory for leg: %s",
                      leg_name.c_str());
    }

    ROS_DEBUG_NAMED(LOGNAME, "Finished planning trajectory for leg: %s", leg_name.c_str());
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
      // Get reference foot states for leg
      const FootState foot_state = referenceState(leg_name, gait_map.at(leg_name).second);
      foot_state_map.emplace(leg_name, foot_state);
    }
  }

  return foot_state_map;
}

FootState FootTrajectoryManager::referenceState(const std::string& leg_name,
                                                double phase) const
{
  const auto search = traj_map_.find(leg_name);
  if (search != traj_map_.end())
  {
    // Time in the trajecotry is a function of the swing phase i.e trajectory(t(phase))
    const auto t = std::clamp(slope_ * phase + y_intercept_, 0.0, 1.0);
    // if (leg_name == "FL")
    // {
    //   std::cout << "t: " << t << " phase: " << phase << std::endl;
    // }

    return traj_map_.at(leg_name).trackTrajectory(t);
  }

  ROS_ERROR_NAMED(LOGNAME,
                  "Failed to find trajectory for leg: %s. May need to re-plan foot "
                  "trajectories.",
                  leg_name.c_str());

  return FootState();
}
}  // namespace quadruped_controller
