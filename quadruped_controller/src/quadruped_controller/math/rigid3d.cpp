/**
 * @file rigid3d.cpp
 * @date 2021-02-28
 * @author Boston Cleek
 * @brief 3D rigid body lie algebra
 */

#include <iostream>
#include <cmath>
#include <quadruped_controller/math/rigid3d.hpp>

namespace quadruped_controller
{
namespace rigid3d
{
using std::cos;
using std::sin;


mat eigen_rotation_to_arma(const Eigen::MatrixXd& R_eigen)
{
  // Read only from R_eigen in memory
  // This is a copy operation for safety
  return mat(R_eigen.data(), R_eigen.rows(), R_eigen.cols());
}

Eigen::MatrixXd arma_rotation_to_eigen(const mat& R_arma)
{
  // Copy data over for safety
  // If Armadillo's memptr() is used this violates the arguements const
  Eigen::MatrixXd R_eigen(3, 3);
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


mat skew_symmetric(const vec& x)
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


Rotation3d Quaternion::rotation() const
{
  return Rotation3d(*this);
}


mat Quaternion::rotationMatrix() const
{
  return Rotation3d(*this).rotationMatrix();
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


/////////////////////////////////////////////////////////
// Rotation
Rotation3d::Rotation3d()
{
}


Rotation3d::Rotation3d(const Quaternion& quaternion) : R_(quaternion.data())
{
}


Rotation3d::Rotation3d(const mat& rotation_matrix)
  : R_(arma_rotation_to_eigen(rotation_matrix))
{
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


/////////////////////////////////////////////////////////
// Transformation


/////////////////////////////////////////////////////////
// Adjoint
}  // namespace rigid3d
}  // namespace quadruped_controller
