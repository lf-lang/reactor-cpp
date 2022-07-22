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

#include "reactor-cpp/base_scheduler.hh"
#include "reactor-cpp/fwd.hh"
#include "reactor-cpp/logical_time.hh"

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

template <class SchedulingPolicy> class Scheduler : public BaseScheduler {
private:
  using worker_t = Worker<SchedulingPolicy>;

  SchedulingPolicy policy_;

  std::vector<worker_t> workers_{};

  std::vector<std::vector<BasePort*>> set_ports_;

  std::atomic<bool> stop_{false};

  auto next() -> bool;
  void set_port_helper(BasePort* port);

public:
  explicit Scheduler(Environment* env);
  Scheduler(Scheduler&&) = delete;
  Scheduler(const Scheduler&) = delete;
  ~Scheduler() override = default;
  auto operator=(Scheduler&&) -> Scheduler& = delete;
  auto operator=(const Scheduler&) -> Scheduler& = delete;

  void start();
  void stop();

  void set_port(BasePort* port) override;

  friend SchedulingPolicy;
};

} // namespace reactor

#include "reactor-cpp/impl/scheduler_impl.hh"

#endif // REACTOR_CPP_SCHEDULER_HH
