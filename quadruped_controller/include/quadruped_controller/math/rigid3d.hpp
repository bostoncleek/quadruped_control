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
#include <tuple>

// Drake
#include <drake/math/rotation_matrix.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>


// Linear Algebra
#include <armadillo>

namespace quadruped_controller
{
namespace math
{
using std::tuple;

using arma::mat;
using arma::vec;
using arma::vec3;

class Rotation3d;

mat eigen_to_arma(const Eigen::MatrixXd& R_eigen);

// vec eigen_to_arma(const Eigen::VectorXd& p_eigen);

Eigen::Matrix3d arma_rotation_to_eigen(const mat& R_arma);

Eigen::Vector3d arma_translation_to_eigen(const vec3& p);

mat skew_symmetric(const vec3& x);


class Quaternion
{
public:
  Quaternion();

  Quaternion(double qw, double qx, double qy, double qz);

  Quaternion(const Eigen::Quaterniond& q);

  Quaternion(const Rotation3d& R);

  Quaternion(const mat& R);

  Quaternion(double angle, const vec3& axis);

  Rotation3d rotation() const;

  mat matrix() const;

  vec3 eulerAngles() const;

  void print(const std::string& msg = "") const;

  bool isUnit() const;

  const Eigen::Quaterniond& data() const
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
  Eigen::Quaterniond q_;
};


class Rotation3d
{
public:
  Rotation3d();

  Rotation3d(const Quaternion& quaternion);

  Rotation3d(const mat& R);

  Rotation3d(double roll, double pitch, double yaw);

  vec3 operator*(const vec3& p) const
  {
    return matrix() * p;
  }

  tuple<vec, double> angleAxis() const;

  vec angleAxisTotal() const;

  mat matrix() const
  {
    return eigen_to_arma(R_.matrix());
  }

  const drake::math::RotationMatrix<double>& data() const 
  {
    return R_;
  }

  Quaternion toQuaternion() const
  {
    return Quaternion(R_.ToQuaternion());
  }

  Eigen::Quaterniond toEigenQuaternion() const
  {
    return R_.ToQuaternion();
  }

  void print(const std::string& msg = "") const
  {
    matrix().print(msg);
  }

private:
  drake::math::RotationMatrix<double> R_;
};


class Transform3d 
{
public:
  Transform3d();

  Transform3d(const vec3& p);

  Transform3d(const drake::math::RigidTransform<double>& T);

  Transform3d(const Quaternion& q, const vec3& p);

  Transform3d(const Rotation3d& R, const vec3& p);

  Transform3d(const mat& R, const vec3& p);

  mat adjoint() const;

  tuple<Quaternion, vec3> components() const;

  Quaternion getQuaternion() const;

  vec3 getTranslation() const;

  Transform3d operator*(const Transform3d& T) const 
  {
    return Transform3d(T_ * T.data());
  }

  vec3 operator*(const vec3& p) const;
  
  const drake::math::RigidTransform<double>& data() const
  {
    return T_;
  }

  mat inverse() const 
  {
    return eigen_to_arma(T_.inverse().GetAsMatrix4());
  }

  mat matrix() const
  {
    return eigen_to_arma(T_.GetAsMatrix4());
  }

  void print(const std::string& msg = "") const
  {
    matrix().print(msg);
  }

private:
  drake::math::RigidTransform<double> T_;

};
}  // namespace math
}  // namespace quadruped_controller
#endif
