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
  , phases_(offset)
  , running_(false)
{
  // Stance phase on domain [0 stance_phase_]
  // Swing phase on domain (stance_angle 1)
  stance_phase_ = t_stance_ / (t_swing_ + t_stance_);
}

GaitScheduler::~GaitScheduler()
{
  stop();
}

void GaitScheduler::start() const
{
  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting GaitScheduler");
  running_ = true;
  worker_ = std::thread([this]() { this->execute(); });
}

void GaitScheduler::stop() const
{
  if (worker_.joinable())
  {
    ROS_INFO_STREAM_NAMED(LOGNAME, "Stopping GaitScheduler");
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
  phases_ = offset_;
}

GaitMap GaitScheduler::schedule() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  GaitMap gait_map;

  gait_map.emplace("RL", std::make_pair(phase(phases_(0)), phases_(0)));
  gait_map.emplace("FL", std::make_pair(phase(phases_(1)), phases_(1)));
  gait_map.emplace("RR", std::make_pair(phase(phases_(2)), phases_(2)));
  gait_map.emplace("FR", std::make_pair(phase(phases_(3)), phases_(3)));

  return gait_map;
}

void GaitScheduler::execute() const
{
  auto start = std::chrono::steady_clock::now();
  while (running_)
  {
    const auto current = std::chrono::steady_clock::now();
    const auto dt = std::chrono::duration<double>(current - start).count();
    start = current;

    update(dt);

    // std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 200 Hz
  }
}

void GaitScheduler::update(double dt) const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  phases_ += 1.0 / (t_swing_ + t_stance_) * dt;

  // wrap phases [0 1)
  phases_(0) = std::fmod(phases_(0), 1.0);
  phases_(1) = std::fmod(phases_(1), 1.0);
  phases_(2) = std::fmod(phases_(2), 1.0);
  phases_(3) = std::fmod(phases_(3), 1.0);
}

LegState GaitScheduler::phase(double phase) const
{
  if ((phase > 0.0 || almost_equal(phase, 0.0)) &&
      (phase < stance_phase_ || almost_equal(phase, stance_phase_)))
  {
    return LegState::stance;
  }

  return LegState::swing;
}
}  // namespace quadruped_controller
