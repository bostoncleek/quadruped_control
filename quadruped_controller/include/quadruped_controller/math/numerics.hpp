/**
 * @file numerics.hpp
 * @date 2021-03-12
 * @author Boston Cleek
 * @brief Numerical utility functions
 */
#ifndef NUMERICS_HPP
#define NUMERICS_HPP

namespace quadruped_controller
{
namespace math
{
constexpr double PI = 3.14159265358979323846;

/**
 * @brief approximately compare two floating-point numbers
 * @param d1 - a number to compare
 * @param d2 - a second number to compare
 * @param epsilon - absolute threshold required for equality
 * @return true if abs(d1 - d2) < epsilon
 */
bool almost_equal(double d1, double d2, double epsilon = 1.0e-12);
}  // namespace math
}  // namespace quadruped_controller
#endif
