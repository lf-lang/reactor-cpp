/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_SCHEDULER_HH
#define REACTOR_CPP_SCHEDULER_HH

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "fwd.hh"
#include "logical_time.hh"

namespace reactor {

template <class SchedulingPolicy> class Worker {
private:
  SchedulingPolicy& policy_;
  const std::size_t identity_{0};
  std::thread thread_{};

  // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
  static thread_local std::size_t current_worker_id_;

  void work() const;
  void execute_reaction(Reaction* reaction) const;

public:
  Worker(SchedulingPolicy& policy, std::size_t identity)
      : policy_{policy}
      , identity_{identity} {}
  Worker(Worker&& worker); // NOLINT(performance-noexcept-move-constructor)
  Worker(const Worker& worker) = delete;
  ~Worker() = default;

  auto operator=(const Worker& worker) -> Worker& = delete;
  auto operator=(Worker&& worker) -> Worker& = delete;

  void start() { thread_ = std::thread(&Worker::work, this); }
  void join() { thread_.join(); }

  [[nodiscard]] auto id() const -> std::size_t { return identity_; }

  static auto current_worker_id() -> unsigned;

  friend SchedulingPolicy;
};

using EventMap = std::map<BaseAction*, std::function<void(void)>>;

template <class SchedulingPolicy> class Scheduler {
private:
  using worker_t = Worker<SchedulingPolicy>;

  SchedulingPolicy policy_;

  const bool using_workers_;
  LogicalTime logical_time_{};

  Environment* environment_;
  std::vector<worker_t> workers_{};

  std::mutex scheduling_mutex_;
  std::unique_lock<std::mutex> scheduling_lock_{scheduling_mutex_, std::defer_lock};
  std::condition_variable cv_schedule_;

  std::mutex lock_event_queue_;
  std::map<Tag, EventMap> event_queue_;

  std::vector<std::vector<BasePort*>> set_ports_;

  std::atomic<bool> stop_{false};

  std::vector<std::vector<Reaction*>> triggered_reactions_;

  auto next() -> bool;
  void set_port_helper(BasePort* port);

public:
  explicit Scheduler(Environment* env);
  Scheduler(Scheduler&&) = delete;
  Scheduler(const Scheduler&) = delete;
  ~Scheduler() = default;
  auto operator=(Scheduler&&) -> Scheduler = delete;
  auto operator=(const Scheduler&) -> Scheduler = delete;

  void schedule_sync(const Tag& tag, BaseAction* action, std::function<void(void)> pre_handler);
  void schedule_async(const Tag& tag, BaseAction* action, std::function<void(void)> pre_handler);

  void inline lock() noexcept { scheduling_lock_.lock(); }
  void inline unlock() noexcept { scheduling_lock_.unlock(); }

  void set_port(BasePort* port);

  [[nodiscard]] inline auto logical_time() const noexcept -> const auto& { return logical_time_; }

  void start();
  void stop();

  friend SchedulingPolicy;
};

} // namespace reactor

#endif // REACTOR_CPP_SCHEDULER_HH
