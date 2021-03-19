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

/** @brief Copy Eigen matrix to Armadillo matrix */
mat eigen_to_arma(const Eigen::MatrixXd& R_eigen);

// vec eigen_to_arma(const Eigen::VectorXd& p_eigen);

/** @brief Copy Armadillo rotation matrix (3x3) to Eigen matrix */
Eigen::Matrix3d arma_rotation_to_eigen(const mat& R_arma);

/** @brief Copy Armadillo translation vecotr to Eigen vector */
Eigen::Vector3d arma_translation_to_eigen(const vec3& p);

/**
 * @brief Compose a skew symmetric matrix
 * @param x - vector
 * @return skew symmetric matrix
 */
mat skew_symmetric(const vec3& x);

/** @brief Rotation in three-dimensional catesian space */
class Rotation3d;

/** @brief Quotient of two directed lines in a three-dimensional space*/
class Quaternion
{
public:
  /** @brief Constructor */
  Quaternion();

  /** @brief Constructor q = qw + qx*i + qy*j + qz*k */
  Quaternion(double qw, double qx, double qy, double qz);

  /**
   * @brief Constructor
   * @param q - Eigen quaternion
   */
  Quaternion(const Eigen::Quaterniond& q);

  /**
   * @brief Constructor
   * @param R - three-dimensional rotation
   */
  Quaternion(const Rotation3d& R);

  /**
   * @brief Constructor
   * @param R - rotation matrix (3x3)
   */
  Quaternion(const mat& R);

  /**
   * @brief Constructor
   * @param angle - rotation magnitude
   * @param axis - axis of rotation
   */
  Quaternion(double angle, const vec3& axis);

  /** @brief Return three-dimensional rotation */
  Rotation3d rotation() const;

  /** @brief Return rotation matrix (3x3) */
  mat matrix() const;

  /** @brief Return roll, pitch, and yaw angles (radians) */
  vec3 eulerAngles() const;

  /** @brief Print contents to stdout */
  void print(const std::string& msg = "") const;

  /** @brief Return true if ||q|| = 1 */
  bool isUnit() const;

  /** @brief Return Eigen quaternion<double> */
  const Eigen::Quaterniond& data() const;

  /** @brief Return qw */
  double w() const;

  /** @brief Return qx */
  double x() const;

  /** @brief Return qy */
  double y() const;

  /** @brief Return qz */
  double z() const;

private:
  Eigen::Quaterniond q_;
};

class Rotation3d
{
public:
  /** @brief Constructor */
  Rotation3d();

  /**
   * @brief Constructor
   * @param quaternion - rotation representaion
   */
  Rotation3d(const Quaternion& quaternion);

  /**
   * @brief Constructor
   * @param R - rotation matrix (3x3)
   */
  Rotation3d(const mat& R);

  /**
   * @brief Constructor
   * @param roll - roation about x-axis (radians)
   * @param pitch - roation about y-axis (radians)
   * @param yaw - roation about z-axis (radians)
   */
  Rotation3d(double roll, double pitch, double yaw);

  /**
   * @brief Rotate a vector
   * @param p - vector
   * @return rotated vector
   */
  vec3 operator*(const vec3& p) const;

  /**
   * @brief Compose angle axis rotation
   * @return rotation axis as a unit vector and angle
   */
  tuple<vec, double> angleAxis() const;

  /**
   * @brief Compose angle axis rotation
   * @return rotation axis
   */
  vec angleAxisTotal() const;

  /** @brief Return rotation matrix (3x3) */
  mat matrix() const;

  /** @brief Return drake rotation matrix */
  const drake::math::RotationMatrix<double>& data() const;

  /** @brief Return rotation as a quaternion */
  Quaternion toQuaternion() const;

  /** @brief Return rotation as a Eigen quaternion */
  Eigen::Quaterniond toEigenQuaternion() const;

  /** @brief Print contents to stdout */
  void print(const std::string& msg = "") const;

private:
  drake::math::RotationMatrix<double> R_;
};

/** @brief Three-dimensional transformation */
class Transform3d
{
public:
  /** @brief Constructor */
  Transform3d();

  /**
   * @brief Constructor
   * @param p - translation
   */
  Transform3d(const vec3& p);

  /**
   * @brief Constructor
   * @param T - drake transformation
   */
  Transform3d(const drake::math::RigidTransform<double>& T);

  /**
   * @brief Constructor
   * @param q - quaternion
   * @param p - translation
   */
  Transform3d(const Quaternion& q, const vec3& p);

  /**
   * @brief Constructor
   * @param R - rotation
   * @param p - translation
   */
  Transform3d(const Rotation3d& R, const vec3& p);

  /**
   * @brief Constructor
   * @param R - rotation matrix (3x3)
   * @param p - translation
   */
  Transform3d(const mat& R, const vec3& p);

  /**
   * @brief Compose adjoint matrix (6x6)
   * @details Assumes the velocity twist is a six-dimensional
   * vector of the form [vy, vy, vz, wx, wy, wz]
   */
  mat adjoint() const;

  /** @brief Return rotation and translation components */
  tuple<Quaternion, vec3> components() const;

  /** @brief Return quaternion component */
  Quaternion getQuaternion() const;

  /** @brief Return translation component */
  vec3 getTranslation() const;

  /**
   * @brief Compose new transformation
   * @param T - transformation to apply
   * @return new transformation
   */
  Transform3d operator*(const Transform3d& T) const;

  /**
   * @brief Compose transformed vector
   * @param p - vector
   * @return transformed vector
   */
  vec3 operator*(const vec3& p) const;

  /** @brief Return drake transformation representation */
  const drake::math::RigidTransform<double>& data() const;

  /** @brief Return inverse homogenous transformation matrix (4x4) */
  mat inverse() const;

  /** @brief Return homogenous transformation matrix (4x4) */
  mat matrix() const;

  /** @brief Print contents to stdout */
  void print(const std::string& msg = "") const;

private:
  drake::math::RigidTransform<double> T_;
};

/** @brief Robot pose */
struct Pose
{
  /** @brief Constructor */
  Pose();

  /**
   * @brief Constructor
   * @param p - translation
   */
  Pose(const vec3& p);

  /**
   * @brief Constructor
   * @param q - quaternion
   */
  Pose(const Quaternion& q);

  /**
   * @brief Constructor
   * @param T - homogenous transformation
   */
  Pose(const Transform3d& T);

  /**
   * @brief Constructor
   * @param q - quaternion
   * @param p - translation
   */
  Pose(const Quaternion& q, const vec3& p);

  /**
   * @brief Constructor
   * @param R - rotation
   * @param p - translation
   */
  Pose(const Rotation3d& R, const vec3& p);

  /**
   * @brief Constructor
   * @param R - rotation matrix (3x3)
   * @param p - translation
   */
  Pose(const mat& R, const vec3& p);

  /** @brief Return pose represented as a homogenous transformation */
  Transform3d transform() const;

  /** @brief Print contents to stdout */
  void print(const std::string& msg = "") const;

  Quaternion orientation;
  vec3 position;  // (x,y,z)
};

}  // namespace math
}  // namespace quadruped_controller
#endif
