/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_TIME_BARRIER_HH
#define REACTOR_CPP_TIME_BARRIER_HH

#include "fwd.hh"
#include "logical_time.hh"
#include "scheduler.hh"
#include "time.hh"
#include <functional>

namespace reactor {

class PhysicalTimeBarrier {
  inline static std::atomic<Duration> last_observed_physical_time_{Duration::zero()}; // NOLINT

public:
  static inline auto try_acquire_tag(const Tag& tag) -> bool {
    // First, we compare against the last observed physical time. This variable
    // serves as a cache for reading the physical clock. Reading from the physical
    // clock can be slow and, thus, this is an optimization that ensures that we
    // only read the clock when it is needed.
    if (tag.time_point().time_since_epoch() < last_observed_physical_time_.load(std::memory_order_acquire)) {
      return true;
    }

    auto physical_time = get_physical_time();
    last_observed_physical_time_.store(physical_time.time_since_epoch(), std::memory_order_release);

    return tag.time_point() < physical_time;
  }

  static inline auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock, Scheduler* scheduler,
                                 const std::function<bool(void)>& abort_waiting) -> bool {
    if (try_acquire_tag(tag)) {
      return true;
    }
    return scheduler->wait_until(lock, tag.time_point(), abort_waiting);
  }
};

class LogicalTimeBarrier {
private:
  LogicalTime released_time_;
  Scheduler* scheduler_;

public:
  LogicalTimeBarrier(Scheduler* scheduler)
      : scheduler_(scheduler) {}

  inline void release_tag(const LogicalTime& tag) {
    auto lock = scheduler_->lock();
    released_time_.advance_to(tag);
  }

  // The caller must hold a lock on the scheduler mutex
  inline auto try_acquire_tag(const Tag& tag) { return tag <= released_time_; }

  inline auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock,
                          const std::function<bool(void)>& abort_waiting) -> bool {
    scheduler_->wait(lock, [this, &tag, &abort_waiting]() { return try_acquire_tag(tag) || abort_waiting(); });
    return !abort_waiting();
  }
};

} // namespace reactor

#endif // REACTOR_CPP_TIME_BARRIER_HH
