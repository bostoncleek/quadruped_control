/**
 * @file numerics.cpp
 * @date 2021-03-12
 * @author Boston Cleek
 * @brief Numerical utility functions
 */

// C++
#include <cmath>

// Quadruped Control
#include <quadruped_controller/math/numerics.hpp>


namespace quadruped_controller
{
namespace math
{
bool almost_equal(double d1, double d2, double epsilon)
{
  return std::fabs(d1 - d2) < epsilon ? true : false;
}


}  // namespace math
}  // namespace quadruped_controller
