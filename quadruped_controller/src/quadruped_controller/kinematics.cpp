/**
 * @file kinematics.cpp
 * @date 2021-09-03
 * @author Boston Cleek
 * @brief Quadruped kinematics
 */

#include <quadruped_controller/kinematics.hpp>
#include <quadruped_controller/math/numerics.hpp>

namespace quadruped_controller
{
using std::atan2;
using std::cos;
using std::pow;
using std::sin;
using std::sqrt;

QuadrupedKinematics::QuadrupedKinematics()
{
  // TODO: Load all these in
  // Vector from base to hip
  const auto xbh = 0.196;
  const auto ybh = 0.050;
  const auto zbh = 0.0;

  // Links between joints
  const auto l1 = 0.077;
  const auto l2 = 0.211;
  const auto l3 = 0.230;
  links_ = { l1, l2, l3 };

  // Base to each hip
  const vec3 trans_rl = { -xbh, ybh, zbh };
  const vec3 trans_fl = { xbh, ybh, zbh };

  const vec3 trans_rr = { -xbh, -ybh, zbh };
  const vec3 trans_fr = { xbh, -ybh, zbh };

  // Left and Right leg link configurations
  const vec3 left_links = { l1, -l2, -l3 };
  const vec3 right_links = { -l1, -l2, -l3 };

  link_map_.emplace("RL", std::pair(trans_rl, left_links));
  link_map_.emplace("FL", std::pair(trans_fl, left_links));
  link_map_.emplace("RR", std::pair(trans_rr, right_links));
  link_map_.emplace("FR", std::pair(trans_fr, right_links));

  // TEST
  // vec q = {0.63, 1.04, -1.60};

  // leg_forward_kinematics(link_map_.at("RL").first, link_map_.at("RL").second,
  // q).print("FK_RL"); leg_forward_kinematics(link_map_.at("FL").first,
  // link_map_.at("FL").second, q).print("FK_FL");

  // leg_forward_kinematics(link_map_.at("RR").first, link_map_.at("RR").second,
  // q).print("FK_RR"); leg_forward_kinematics(link_map_.at("FR").first,
  // link_map_.at("FR").second, q).print("FK_FR");

  // leg_jacobian(link_map_.at("RL").second, q).print("J_RL");
  // leg_jacobian(link_map_.at("FL").second, q).print("J_FL");

  // leg_jacobian(link_map_.at("RR").second, q).print("J_RR");
  // leg_jacobian(link_map_.at("FR").second, q).print("J_FR");
}

mat QuadrupedKinematics::forwardKinematics(const vec& q) const
{
  // Follows joint_state topic format: RL, FL, RR, FR
  // Foot position relative to base frame
  // Each column is a foot position (x,y,z)
  mat ft_p(3, 4);
  ft_p.col(0) = forwardKinematics("RL", q.rows(0, 2));
  ft_p.col(1) = forwardKinematics("FL", q.rows(3, 5));
  ft_p.col(2) = forwardKinematics("RR", q.rows(6, 8));
  ft_p.col(3) = forwardKinematics("FR", q.rows(9, 11));

  return ft_p;
}

vec3 QuadrupedKinematics::forwardKinematics(const std::string& leg_name,
                                            const vec3& q) const
{
  const vec3 trans_bh = link_map_.at(leg_name).first;
  const vec3 links = link_map_.at(leg_name).second;

  const auto l1 = links(0);
  const auto l2 = links(1);
  const auto l3 = links(2);

  const auto t1 = q(0);
  const auto t2 = q(1);
  const auto t3 = q(2);

  vec3 foot_position;
  foot_position(0) = l2 * sin(t2) + l3 * sin(t2 + t3) + trans_bh(0);
  foot_position(1) =
      l1 * cos(t1) - l2 * sin(t1) * cos(t2) - l3 * sin(t1) * cos(t2 + t3) + trans_bh(1);
  foot_position(2) =
      l1 * sin(t1) + l2 * cos(t1) * cos(t2) + l3 * cos(t1) * cos(t2 + t3) + trans_bh(2);

  return foot_position;
}

FootholdMap
QuadrupedKinematics::forwardKinematics(const JointStatesMap& joint_states_map) const
{
  FootholdMap foot_hold_map;
  for (const auto& [leg_name, joint_states] : joint_states_map)
  {
    foot_hold_map.emplace(leg_name, forwardKinematics(leg_name, joint_states.q));
  }

  return foot_hold_map;
}

vec3 QuadrupedKinematics::legInverseKinematics(const std::string& leg_name,
                                               const vec3& foothold) const
{
  // position of foot relative to hip
  const vec3 ft_p = foothold - link_map_.at(leg_name).first;

  const auto x = ft_p(0);
  const auto y = ft_p(1);
  const auto z = ft_p(2);

  const auto l1 = links_(0);
  const auto l2 = links_(1);
  const auto l3 = links_(2);

  auto d = (x * x + y * y + z * z - l1 * l1 - l2 * l2 - l3 * l3) / (2.0 * l2 * l3);

  if (d > 1.0)
  {
    d = 1.0;
  }

  auto sqrt_component = y * y + z * z - l1 * l1;
  if (sqrt_component < 0.0)
  {
    sqrt_component = 0.0;
  }

  vec3 q;

  // TODO: make sure leg name is valid
  if (leg_name == "FR" || leg_name == "RR")
  {
    q(0) = atan2(z, y) + atan2(sqrt(sqrt_component), -l1);
  }
  else
  {
    q(0) = -(atan2(z, -y) + atan2(sqrt(sqrt_component), -l1));
  }

  q(2) = atan2(-sqrt(1.0 - d * d), d);
  q(1) = -atan2(x, sqrt(sqrt_component)) - atan2(l3 * sin(q(2)), l2 + l3 * cos(q(2)));

  return q;
}

mat33 QuadrupedKinematics::legJacobian(const std::string& leg_name, const vec3& q) const
{
  const vec3 links = link_map_.at(leg_name).second;

  const auto l1 = links(0);
  const auto l2 = links(1);
  const auto l3 = links(2);

  const auto t1 = q(0);
  const auto t2 = q(1);
  const auto t3 = q(2);

  mat jac(3, 3);
  jac(0, 0) = 0.0;
  jac(0, 1) = l2 * cos(t2) + l3 * cos(t2 + t3);
  jac(0, 2) = l3 * cos(t2 + t3);

  jac(1, 0) = -l1 * sin(t1) - l2 * cos(t1) * cos(t2) - l3 * cos(t1) * cos(t2 + t3);
  jac(1, 1) = (l2 * sin(t2) + l3 * sin(t2 + t3)) * sin(t1);
  jac(1, 2) = l3 * sin(t1) * sin(t2 + t3);

  jac(2, 0) = l1 * cos(t1) - l2 * sin(t1) * cos(t2) - l3 * sin(t1) * cos(t2 + t3);
  jac(2, 1) = -(l2 * sin(t2) + l3 * sin(t2 + t3)) * cos(t1);
  jac(2, 2) = -l3 * sin(t2 + t3) * cos(t1);

  return jac;
}

mat33 QuadrupedKinematics::legJacobianInverse(const std::string& leg_name,
                                              const vec3& q) const
{
  mat33 Jinv;
  const mat33 J = legJacobian(leg_name, q);  
  if (!arma::inv(Jinv, J))
  {
    if (!arma::pinv(Jinv, J))
    {
      Jinv = J.t();
    }
  }

  return Jinv;
}

vec QuadrupedKinematics::jacobianTransposeControl(const vec& q, const vec& f) const
{
  vec tau(12);

  tau.rows(0, 2) = legJacobianInverse("RL", q.rows(0, 2)) * f.rows(0, 2);
  tau.rows(3, 5) = legJacobianInverse("FL", q.rows(3, 5)) * f.rows(3, 5);
  tau.rows(6, 8) = legJacobianInverse("RR", q.rows(6, 8)) * f.rows(6, 8);
  tau.rows(9, 11) = legJacobianInverse("FR", q.rows(9, 11)) * f.rows(9, 11);

  return tau;
}

TorqueMap
QuadrupedKinematics::jacobianTransposeControl(const JointStatesMap& joint_states_map,
                                              const ForceMap& force_map) const
{
  TorqueMap torque_map;
  for (const auto& [leg_name, force] : force_map)
  {
    const mat33 J = legJacobian(leg_name, joint_states_map.at(leg_name).q);
    const vec3 tau = J.t() * force;
    torque_map.emplace(leg_name, tau);
  }

  return torque_map;
}

}  // namespace quadruped_controller
