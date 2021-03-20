/**
 * @file gait.cpp
 * @date 2021-03-20
 * @author Boston Cleek
 * @brief Gait generator
 */

// C++
#include <chrono>

#include <ros/console.h>

#include <quadruped_controller/gait.hpp>
#include <quadruped_controller/math/numerics.hpp>

namespace quadruped_controller
{
using math::almost_equal;
using math::normalize_angle_2PI;
using math::PI;

static const std::string LOGNAME = "Gait Scheduler";

GaitScheduler::GaitScheduler(double t_swing, double t_stance, const vec& offset)
  : t_swing_(t_swing)
  , t_stance_(t_stance)
  , offset_(offset)
  , positions_(offset)
  , running_(false)
{
  // Circumference maps to gait period T = 2*PI*r
  // Stance phase on domain [0 stance_angle]
  stance_angle_ = (t_stance_ / (t_swing_ + t_stance_)) * 2.0 * PI;

  // Swing phase on domain (stance_angle 2PI)
  // double swing_angle = 2.0 * PI - stance_angle_;

  // std::cout << "Stance angle: " << stance_angle_ << std::endl;
  // std::cout << "Swing angle: " << swing_angle_ << std::endl;
}

GaitScheduler::~GaitScheduler()
{
  stop();
}

void GaitScheduler::start() const
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting GaitScheduler");
  running_ = true;
  worker_ = std::thread{ [this]() { this->execute(); } };
}

void GaitScheduler::stop() const
{
  if (worker_.joinable())
  {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Starting GaitScheduler");
    running_ = false;
    worker_.join();
    return;
  }

  ROS_ERROR_STREAM_NAMED(LOGNAME, "GaitScheduler thread is not joinable.");
}

void GaitScheduler::reset() const
{
  if (running_)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Cannot reset GaitScheduler while running.");
    return;
  }

  ROS_INFO_STREAM_NAMED(LOGNAME, "Resetting GaitScheduler");
  positions_ = offset_;
}

gait GaitScheduler::schedule() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  gait gait_map;

  gait_map.emplace(std::make_pair("RL", phase(positions_(0))));
  gait_map.emplace(std::make_pair("FL", phase(positions_(1))));
  gait_map.emplace(std::make_pair("RR", phase(positions_(2))));
  gait_map.emplace(std::make_pair("FR", phase(positions_(3))));

  return gait_map;
}

void GaitScheduler::execute() const
{
  auto start = std::chrono::steady_clock::now();
  while (running_)
  {
    const auto current = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(current - start).count();
    start = current;

    update(dt);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz
  }
}

void GaitScheduler::update(double dt) const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  positions_ += 2.0 * PI / (t_swing_ + t_stance_) * dt;

  positions_(0) = normalize_angle_2PI(positions_(0));
  positions_(1) = normalize_angle_2PI(positions_(1));
  positions_(2) = normalize_angle_2PI(positions_(2));
  positions_(3) = normalize_angle_2PI(positions_(3));
}

LegState GaitScheduler::phase(double angle) const
{
  if ((angle > 0.0 || almost_equal(angle, 0.0)) &&
      (angle < stance_angle_ || almost_equal(angle, stance_angle_)))
  {
    return LegState::stance;
  }

  return LegState::swing;
}
}  // namespace quadruped_controller
