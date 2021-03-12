/**
 * @file kinematics.hpp
 * @date 2021-09-04
 * @author Boston Cleek
 * @brief Quadruped kinematics
 */
#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

// C++
#include <map>
#include <vector>
#include <string>
#include <functional>
#include <cmath>

// Linear Algebra
#include <armadillo>

namespace quadruped_controller
{
using arma::mat;
using arma::vec;

using std::map;
using std::string;
using std::vector;

using std::cos;
using std::sin;

// using JacobianFunc = std::function<mat(const vec&, const vec&)>;

mat leg_jacobian(const vec& links, const vec& joints)
{
  const auto l1 = links(0);
  const auto l2 = links(1);
  const auto l3 = links(2);

  const auto t1 = joints(0);
  const auto t2 = joints(1);
  const auto t3 = joints(2);

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


vec leg_forward_kinematics(const vec& trans_bh, const vec& links, const vec& joints)
{
  const auto l1 = links(0);
  const auto l2 = links(1);
  const auto l3 = links(2);

  const auto t1 = joints(0);
  const auto t2 = joints(1);
  const auto t3 = joints(2);

  vec foot_position(3);
  foot_position(0) = l2 * sin(t2) + l3 * sin(t2 + t3) + trans_bh(0);
  foot_position(1) =
      l1 * cos(t1) - l2 * sin(t1) * cos(t2) - l3 * sin(t1) * cos(t2 + t3) + trans_bh(1);
  foot_position(2) =
      l1 * sin(t1) + l2 * cos(t1) * cos(t2) + l3 * cos(t1) * cos(t2 + t3) + trans_bh(2);

  return foot_position;
}


class QuadrupedKinematics
{
public:
  QuadrupedKinematics();

  mat forwardKinematics(const vec& q) const;

  vec jacobianTransposeControl(const vec& q, const vec& f);

private:
  // Map leg name to  leg link config/translation base to hip
  map<string, std::pair<vec, vec>> link_map_;
};


}  // namespace quadruped_controller

#endif
