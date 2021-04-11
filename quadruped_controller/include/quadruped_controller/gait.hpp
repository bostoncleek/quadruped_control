/**
 * @file gait.hpp
 * @date 2021-03-20
 * @author Boston Cleek
 * @brief Gait generator
 */
#ifndef GAIT_HPP
#define GAIT_HPP

#include <map>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

#include <quadruped_controller/types.hpp>

namespace quadruped_controller
{
/** @brief Compose a pure stance gait with all phases set to zero */
GaitMap make_stance_gait();

/** @brief Scheduler leg swing and stance phases*/
class GaitScheduler
{
public:
  /**
   * @brief Constructor
   * @param t_swing - leg swing time (s)
   * @param t_stance - leg stance time (s)
   * @param offset - phase offset for legs [RL FL RR FR] on domain [0 1)
   * @details The gait is periodic mapping to a domain [0 1).
   */
  GaitScheduler(double t_swing, double t_stance, const vec& offset);

  /** @brief Destructor */
  virtual ~GaitScheduler();

  /** @brief Starts periodic gait */
  void start() const;

  /** @brief Stops periodic gait */
  void stop() const;

  /**
   * @brief Reset gait schedule to phase offsets
   * @details A call to stop() must be made first
   */
  void reset() const;

  /** @brief Get the current gait schedule */
  GaitMap schedule() const;

private:
  /** @brief Run gait schedule in a separate thread */
  void execute() const;

  /**
   * @brief Update phases
   * @param dt - time lapse since last call (s)
   */
  void update(double dt) const;

  /**
   * @brief Compose state of leg
   * @param phase - current leg phase
   * @return leg is either swing or stance
   * @details If phase : [0 stance_phase] leg is in stance.
   * Once the phase > stance_phase the leg is in swing.
   */
  LegState phase(double angle) const;

private:
  double t_swing_;       // swing time (s)
  double t_stance_;      // stance time (s)
  double stance_phase_;  // stance phase

  vec offset_;          // phase offsets for legs [RL FL RR FR]
  mutable vec phases_;  // phases for legs [RL FL RR FR]

  mutable std::atomic_bool running_;  // true when scheduler started
  mutable std::thread worker_;        // runs phase updates
  mutable std::mutex mutex_;          // protect positions_
};

}  // namespace quadruped_controller
#endif
