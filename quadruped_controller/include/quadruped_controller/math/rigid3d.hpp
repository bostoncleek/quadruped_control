/**
 * @file rigid3d.hpp
 * @date 2021-02-28
 * @author Boston Cleek
 * @brief 3D rigid body lie algebra
 */
#ifndef RIGID3D_HPP
#define RIGID3D_HPP

// C++
#include <iosfwd>
// Drake
#include <drake/math/rotation_matrix.h>
#include <drake/math/rigid_transform.h>

// Linear Algebra
#include <armadillo>

namespace quadruped_controller
{
namespace rigid3d
{
using arma::mat;
using arma::vec;

// using drake::math::RotationMatrix;
// using drake::math::RigidTransform;
// // using drake::Quaternion;
// using drake::Matrix3;

// using Eigen::MatrixXd;
// using Eigen::Map;
// using Eigen::VectorXd;


mat eigen_rotation_to_arma(const Eigen::MatrixXd& R_eigen);

Eigen::MatrixXd arma_rotation_to_eigen(const mat& R_arma);


class Rotation3d;

class Quaternion
{
public:
  Quaternion();

  Quaternion(double qw, double qx, double qy, double qz);

  Quaternion(const Eigen::Quaterniond& q);

  Rotation3d rotation() const;

  mat rotationMatrix() const;

  void print(const std::string& msg = "") const;

  const drake::Quaternion<double>& data() const
  {
    return q_;
  }

  double w() const
  {
    return q_.w();
  }

  double x() const
  {
    return q_.x();
  }

  double y() const
  {
    return q_.y();
  }

  double z() const
  {
    return q_.z();
  }

private:
  drake::Quaternion<double> q_;
};


class Rotation3d
{
public:
  Rotation3d();

  Rotation3d(const Quaternion& quaternion);

  Rotation3d(const mat& rotation_matrix);

  mat rotationMatrix() const
  {
    return eigen_rotation_to_arma(R_.matrix());
  }

  Quaternion toQuaternion() const
  {
    return Quaternion(R_.ToQuaternion());
  }

  void print(const std::string& msg = "") const
  {
    rotationMatrix().print(msg);
  }

private:
  drake::math::RotationMatrix<double> R_;
};
}  // namespace rigid3d
}  // namespace quadruped_controller
#endif
