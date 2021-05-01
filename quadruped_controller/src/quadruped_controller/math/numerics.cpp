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

double normalize_angle_2PI(double angle)
{
  // floating point remainder is essentially fmod
  const auto q = std::floor(angle / (2.0 * PI));
  angle -= q * 2.0 * PI;

  if (angle < 0.0)
  {
    angle += 2.0 * PI;
  }

  return angle;
}

double normalize_angle_PI(double rad)
{
  // floating point remainder essentially this is fmod
  const auto q = std::floor((rad + PI) / (2.0 * PI));
  rad = (rad + PI) - q * 2.0 * PI;

  if (rad < 0)
  {
    rad += 2.0 * PI;
  }

  return (rad - PI);
}

// vec normalize_angle_2PI(vec angles)
// {
//   vec wrapped(angles.size());
//   for (unsigned int i = 0; i < angles.size(); i++)
//   {
//     wrapped(i) = normalize_angle_2PI(angles(i));
//   }

//   return wrapped;
// }

vec3 normalize_angle_2PI(vec3 angles)
{
  vec3 wrapped;
  wrapped(0) = normalize_angle_2PI(angles(0));
  wrapped(1) = normalize_angle_2PI(angles(1));
  wrapped(2) = normalize_angle_2PI(angles(2));

  return wrapped;
}

vec3 normalize_angle_PI(vec3 angles)
{
  vec3 wrapped;
  wrapped(0) = normalize_angle_PI(angles(0));
  wrapped(1) = normalize_angle_PI(angles(1));
  wrapped(2) = normalize_angle_PI(angles(2));

  return wrapped;
}
}  // namespace math
}  // namespace quadruped_controller
