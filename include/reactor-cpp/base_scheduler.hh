/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_BASE_SCHEDULER_HH
#define REACTOR_CPP_BASE_SCHEDULER_HH

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>

#include "reactor-cpp/environment.hh"
#include "reactor-cpp/fwd.hh"
#include "reactor-cpp/logical_time.hh"

namespace reactor {

using EventMap = std::map<BaseAction*, std::function<void(void)>>;

class BaseScheduler {
protected:
  Environment* environment_; // NOLINT

  LogicalTime logical_time_{}; // NOLINT
  const bool using_workers_;   // NOLINT

  std::mutex scheduling_mutex_;                                                      // NOLINT
  std::unique_lock<std::mutex> scheduling_lock_{scheduling_mutex_, std::defer_lock}; // NOLINT
  std::condition_variable cv_schedule_;                                              // NOLINT

  std::mutex lock_event_queue_;         // NOLINT
  std::map<Tag, EventMap> event_queue_; // NOLINT

public:
  BaseScheduler(Environment* env);
  virtual ~BaseScheduler() = default;
  BaseScheduler(const BaseScheduler&) = delete;
  BaseScheduler(BaseScheduler&&) = delete;
  auto operator=(const BaseScheduler&) -> BaseScheduler& = delete;
  auto operator=(BaseScheduler&&) -> BaseScheduler& = delete;

  [[nodiscard]] inline auto logical_time() const noexcept -> const auto& { return logical_time_; }
  void schedule_sync(const Tag& tag, BaseAction* action, std::function<void(void)> pre_handler);
  void schedule_async(const Tag& tag, BaseAction* action, std::function<void(void)> pre_handler);

  virtual void set_port(BasePort* port) = 0;

  void inline lock() noexcept { scheduling_lock_.lock(); }
  void inline unlock() noexcept { scheduling_lock_.unlock(); }
};

} // namespace reactor

#endif // REACTOR_CPP_BASE_SCHEDULER_HH
