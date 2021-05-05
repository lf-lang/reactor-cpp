/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "fwd.hh"
#include "logical_time.hh"
#include "semaphore.hh"

namespace reactor {

class Scheduler;

class Worker {
 public:
  Scheduler& scheduler;
  const unsigned id{0};
  std::thread thread;

  static std::atomic<unsigned> running_workers;
  static std::atomic<bool> terminate;
  static Semaphore work_semaphore;

  void work();
  void process_ready_reactions();

 public:
  Worker(Scheduler& scheduler, unsigned id)
      : scheduler{scheduler}, id{id}, thread{} {}
  Worker(Worker&& w);
  Worker(const Worker&) = delete;

  void start_thread() { thread = std::thread(&Worker::work, this); }
  void join_thread() { thread.join(); }

  static void terminate_all_workers(unsigned count);
  static void wakeup_workers(unsigned count);
};

class Scheduler {
 public:
  using EventMap = std::map<BaseAction*, std::function<void(void)>>;

 private:
  const bool using_workers;
  LogicalTime _logical_time{};

  Environment* _environment;
  std::vector<Worker> workers{};

  std::mutex m_schedule;
  std::unique_lock<std::mutex> schedule_lock{m_schedule, std::defer_lock};
  std::condition_variable cv_schedule;

  std::mutex m_event_queue;
  std::map<Tag, std::unique_ptr<EventMap>> event_queue;
  std::vector<BasePort*> set_ports;

  std::mutex m_reaction_queue;
  std::vector<std::vector<Reaction*>> reaction_queue;
  unsigned reaction_queue_pos{std::numeric_limits<unsigned>::max()};

  std::vector<Reaction*> ready_reactions;
  std::atomic<unsigned> num_ready_reactions{0};

  void schedule();
  void schedule_ready_reactions();

  void next();

  void set_port_helper(BasePort* p);

  std::atomic<bool> _stop{false};
  bool continue_execution{true};

 public:
  Scheduler(Environment* env);
  ~Scheduler();

  void schedule_sync(const Tag& tag,
                     BaseAction* action,
                     std::function<void(void)> pre_handler);
  void schedule_async(const Tag& tag,
                      BaseAction* action,
                      std::function<void(void)> pre_handler);

  void lock() { schedule_lock.lock(); }
  void unlock() { schedule_lock.unlock(); }

  void set_port(BasePort*);

  const LogicalTime& logical_time() const { return _logical_time; }

  void start();

  void stop();

  friend Worker;
};

}  // namespace reactor
