/**
 * @file numerics.hpp
 * @date 2021-03-12
 * @author Boston Cleek
 * @brief Numerical utility functions
 */
#ifndef NUMERICS_HPP
#define NUMERICS_HPP

// Linear Algebra
#include <armadillo>

namespace quadruped_controller
{
namespace math
{
using arma::vec;
using arma::vec3;

constexpr double PI = 3.14159265358979323846;

/**
 * @brief approximately compare two floating-point numbers
 * @param d1 - a number to compare
 * @param d2 - a second number to compare
 * @param epsilon - absolute threshold required for equality
 * @return true if abs(d1 - d2) < epsilon
 */
bool almost_equal(double d1, double d2, double epsilon = 1.0e-12);

/**
 * @brief Normalize angle [0 2PI)
 * @param angle - radians
 * @return normalized angle
 */
double normalize_angle_2PI(double angle);

/**
 * @brief Normalize angle [0 PI)
 * @param angle - radians
 * @return normalized angle
 */
double normalize_angle_PI(double rad);

// /**
//  * @brief Normalize angle [0 2PI)
//  * @param angles - vector in radians
//  * @return vector of normalized angles
//  */
// vec normalize_angle_2PI(vec angles);

/**
 * @brief Normalize angle [0 2PI)
 * @param angles - vector in radians
 * @return vector of normalized angles
 */
vec3 normalize_angle_2PI(vec3 angles);

/**
 * @brief Normalize angle [0 PI)
 * @param angles - vector in radians
 * @return vector of normalized angles
 */
vec3 normalize_angle_PI(vec3 angles);
}  // namespace math
}  // namespace quadruped_controller
#endif
