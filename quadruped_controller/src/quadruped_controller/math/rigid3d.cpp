/**
 * @file rigid3d.cpp
 * @date 2021-02-28
 * @author Boston Cleek
 * @brief 3D rigid body lie algebra
 */

// C++
#include <iostream>
#include <cmath>

// Quadruped Control
#include <quadruped_controller/math/rigid3d.hpp>
#include <quadruped_controller/math/numerics.hpp>

namespace quadruped_controller
{
namespace math
{
using std::cos;
using std::sin;

mat eigen_to_arma(const Eigen::MatrixXd& R_eigen)
{
  // Read only from R_eigen in memory
  // This is a copy operation for safety
  return mat(R_eigen.data(), R_eigen.rows(), R_eigen.cols());
}

// vec eigen_to_arma(const Eigen::VectorXd& p_eigen)
// {
//   return mat(p_eigen.data(), p_eigen.rows(), p_eigen.cols());
// }

Eigen::Matrix3d arma_rotation_to_eigen(const mat& R_arma)
{
  // Copy data over for safety
  // If Armadillo's memptr() is used
  // this violates the input arguments const
  Eigen::Matrix3d R_eigen;
  R_eigen(0, 0) = R_arma(0, 0);
  R_eigen(0, 1) = R_arma(0, 1);
  R_eigen(0, 2) = R_arma(0, 2);

  R_eigen(1, 0) = R_arma(1, 0);
  R_eigen(1, 1) = R_arma(1, 1);
  R_eigen(1, 2) = R_arma(1, 2);

  R_eigen(2, 0) = R_arma(2, 0);
  R_eigen(2, 1) = R_arma(2, 1);
  R_eigen(2, 2) = R_arma(2, 2);

  return R_eigen;
}

Eigen::Vector3d arma_translation_to_eigen(const vec3& p)
{
  return Eigen::Vector3d(p(0), p(1), p(2));
}

mat skew_symmetric(const vec3& x)
{
  mat skew(3, 3, arma::fill::zeros);
  // upper
  skew(0, 1) = -x(2);
  skew(0, 2) = x(1);
  skew(1, 2) = -x(0);
  // lower
  skew(1, 0) = x(2);
  skew(2, 0) = -x(1);
  skew(2, 1) = x(0);

  return skew;
}

/////////////////////////////////////////////////////////
// Quaternion
Quaternion::Quaternion() : q_(1., 0., 0., 0.)
{
}

Quaternion::Quaternion(double qw, double qx, double qy, double qz) : q_(qw, qx, qy, qz)
{
}

Quaternion::Quaternion(const Eigen::Quaterniond& q) : q_(q)
{
}

Quaternion::Quaternion(const Rotation3d& R) : q_(R.toEigenQuaternion())
{
}

Quaternion::Quaternion(const mat& R) : q_(Rotation3d(R).toEigenQuaternion())
{
}

Quaternion::Quaternion(double angle, const vec3& axis)
  : q_(Eigen::AngleAxisd(angle, arma_translation_to_eigen(axis)))
{
}

Rotation3d Quaternion::rotation() const
{
  return Rotation3d(*this);
}

mat Quaternion::matrix() const
{
  return Rotation3d(*this).matrix();
}

vec3 Quaternion::eulerAngles() const
{
  const drake::math::RollPitchYaw angles(q_);
  return { angles.roll_angle(), angles.pitch_angle(), angles.yaw_angle() };
}

void Quaternion::print(const std::string& msg) const
{
  if (!msg.empty())
  {
    std::cout << msg << "\n";
  }
  std::cout << "qw: " << q_.w() << " qx: " << q_.x() << " qy: " << q_.y()
            << " qz: " << q_.z() << std::endl;
}

bool Quaternion::isUnit() const
{
  const auto norm =
      std::sqrt(q_.x() * q_.x() + q_.y() * q_.y() + q_.z() * q_.z() + q_.w() * q_.w());

  if (almost_equal(norm, 1.0))
  {
    return true;
  }

  return false;
}

const Eigen::Quaterniond& Quaternion::data() const
{
  return q_;
}

double Quaternion::w() const
{
  return q_.w();
}

double Quaternion::x() const
{
  return q_.x();
}

double Quaternion::y() const
{
  return q_.y();
}

double Quaternion::z() const
{
  return q_.z();
}

/////////////////////////////////////////////////////////
// Rotation
Rotation3d::Rotation3d()
{
}

Rotation3d::Rotation3d(const Quaternion& quaternion) : R_(quaternion.data())
{
}

Rotation3d::Rotation3d(const mat& R) : R_(arma_rotation_to_eigen(R))
{
}

Rotation3d::Rotation3d(double roll, double pitch, double yaw)
  : R_(drake::math::RollPitchYaw<double>(roll, pitch, yaw))
{
}

vec3 Rotation3d::operator*(const vec3& p) const
{
  return matrix() * p;
}

tuple<vec, double> Rotation3d::angleAxis() const
{
  const Eigen::AngleAxisd aa_eigen = R_.ToAngleAxis();
  const vec axis = { aa_eigen.axis()(0), aa_eigen.axis()(1), aa_eigen.axis()(2) };
  return { axis, aa_eigen.angle() };
}

vec Rotation3d::angleAxisTotal() const
{
  const Eigen::AngleAxisd aa_eigen = R_.ToAngleAxis();
  const Eigen::Vector3d aa_tot_eigen = aa_eigen.axis() * aa_eigen.angle();
  return { aa_tot_eigen(0), aa_tot_eigen(1), aa_tot_eigen(2) };
}

mat Rotation3d::matrix() const
{
  return eigen_to_arma(R_.matrix());
}

const drake::math::RotationMatrix<double>& Rotation3d::data() const
{
  return R_;
}

Quaternion Rotation3d::toQuaternion() const
{
  return Quaternion(R_.ToQuaternion());
}

Eigen::Quaterniond Rotation3d::toEigenQuaternion() const
{
  return R_.ToQuaternion();
}

void Rotation3d::print(const std::string& msg) const
{
  matrix().print(msg);
}

/////////////////////////////////////////////////////////
// Transformation
Transform3d::Transform3d()
{
}

Transform3d::Transform3d(const vec3& p) : T_(arma_translation_to_eigen(p))
{
}

Transform3d::Transform3d(const drake::math::RigidTransform<double>& T) : T_(T)
{
}

Transform3d::Transform3d(const Quaternion& q, const vec3& p)
  : T_(q.data(), arma_translation_to_eigen(p))
{
}

Transform3d::Transform3d(const Rotation3d& R, const vec3& p)
  : T_(R.data(), arma_translation_to_eigen(p))
{
}

Transform3d::Transform3d(const mat& R, const vec3& p)
  : T_(Quaternion(R).data(), arma_translation_to_eigen(p))
{
}

mat Transform3d::adjoint() const
{
  // Assumes twist is [vy, vy, vz, wx, wy, wz]
  mat AdT(6, 6, arma::fill::zeros);
  const mat T = eigen_to_arma(T_.GetAsMatrix34());
  const mat R_transpose = T.submat(0, 0, 2, 2).t();

  AdT.submat(0, 0, 2, 2) = R_transpose;
  AdT.submat(3, 3, 5, 5) = R_transpose;
  AdT.submat(0, 3, 2, 5) = -1.0 * R_transpose * skew_symmetric(T.col(3));

  return AdT;
}

tuple<Quaternion, vec3> Transform3d::components() const
{
  const mat T = eigen_to_arma(T_.GetAsMatrix34());
  return std::make_pair(Quaternion(T.submat(0, 0, 2, 2)), T.submat(0, 3, 2, 3));
}

Quaternion Transform3d::getQuaternion() const
{
  const mat T = eigen_to_arma(T_.GetAsMatrix34());
  return Quaternion(T.submat(0, 0, 2, 2));
}

vec3 Transform3d::getTranslation() const
{
  const Eigen::Vector3d t_eigen = T_.translation();
  return { t_eigen(0), t_eigen(1), t_eigen(2) };
}

Transform3d Transform3d::operator*(const Transform3d& T) const
{
  return Transform3d(T_ * T.data());
}

vec3 Transform3d::operator*(const vec3& p) const
{
  const mat T = eigen_to_arma(T_.GetAsMatrix34());
  // rotation followed by translation
  return T.submat(0, 0, 2, 2) * p + T.col(3);
}

const drake::math::RigidTransform<double>& Transform3d::data() const
{
  return T_;
}

mat Transform3d::inverse() const
{
  return eigen_to_arma(T_.inverse().GetAsMatrix4());
}

mat Transform3d::matrix() const
{
  return eigen_to_arma(T_.GetAsMatrix4());
}

void Transform3d::print(const std::string& msg) const
{
  matrix().print(msg);
}

/////////////////////////////////////////////////////////
// Pose
Pose::Pose() : position(arma::fill::zeros)
{
}

Pose::Pose(const vec3& p) : position(p)
{
}

Pose::Pose(const Quaternion& q) : orientation(q)
{
}

Pose::Pose(const Transform3d& T)
  : orientation(T.getQuaternion()), position(T.getTranslation())
{
}

Pose::Pose(const Quaternion& q, const vec3& p) : orientation(q), position(p)
{
}

Pose::Pose(const Rotation3d& R, const vec3& p) : orientation(R), position(p)
{
}

Pose::Pose(const mat& R, const vec3& p) : orientation(R), position(p)
{
}

Transform3d Pose::transform() const
{
  return Transform3d(orientation, position);
}

void Pose::print(const std::string& msg) const
{
  if (!msg.empty())
  {
    std::cout << msg << "\n";
  }
  orientation.print("orientation: ");
  position.print("position: ");
}
}  // namespace math
}  // namespace quadruped_controller
