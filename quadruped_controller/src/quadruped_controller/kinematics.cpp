/**
 * @file kinematics.cpp
 * @date 2021-09-04
 * @author Boston Cleek
 * @brief Quadruped kinematics
 */

#include <quadruped_controller/kinematics.hpp>


namespace quadruped_controller
{
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

  // Base to each hip
  const vec trans_rl = { -xbh, ybh, zbh };
  const vec trans_fl = { xbh, ybh, zbh };

  const vec trans_rr = { -xbh, -ybh, zbh };
  const vec trans_fr = { xbh, -ybh, zbh };

  // Left and Right leg link configurations
  const vec left_links = { l1, -l2, -l3 };
  const vec right_links = { -l1, -l2, -l3 };

  link_map_.emplace(std::make_pair("RL", std::make_pair(trans_rl, left_links)));
  link_map_.emplace(std::make_pair("FL", std::make_pair(trans_fl, left_links)));

  link_map_.emplace(std::make_pair("RR", std::make_pair(trans_rr, right_links)));
  link_map_.emplace(std::make_pair("FR", std::make_pair(trans_fr, right_links)));

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
  ft_p.col(0) = leg_forward_kinematics(link_map_.at("RL").first,
                                       link_map_.at("RL").second, q.rows(0, 2));
  ft_p.col(1) = leg_forward_kinematics(link_map_.at("FL").first,
                                       link_map_.at("FL").second, q.rows(3, 5));
  ft_p.col(2) = leg_forward_kinematics(link_map_.at("RR").first,
                                       link_map_.at("RR").second, q.rows(6, 8));
  ft_p.col(3) = leg_forward_kinematics(link_map_.at("FR").first,
                                       link_map_.at("FR").second, q.rows(9, 11));

  return ft_p;
}


vec QuadrupedKinematics::jacobianTransposeControl(const vec& q, const vec& f)
{
  vec tau(12);

  const mat j_rl = leg_jacobian(link_map_.at("RL").second, q.rows(0, 2));
  const mat j_fl = leg_jacobian(link_map_.at("FL").second, q.rows(3, 5));
  const mat j_rr = leg_jacobian(link_map_.at("RR").second, q.rows(6, 8));
  const mat j_fr = leg_jacobian(link_map_.at("FR").second, q.rows(9, 11));

  tau.rows(0, 2) = j_rl.t() * f.rows(0, 2);
  tau.rows(3, 5) = j_fl.t() * f.rows(3, 5);
  tau.rows(6, 8) = j_rr.t() * f.rows(6, 8);
  tau.rows(9, 11) = j_fr.t() * f.rows(9, 11);

  return tau;
}
}  // namespace quadruped_controller
