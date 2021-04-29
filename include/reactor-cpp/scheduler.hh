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
#include <semaphore>
#include <set>
#include <thread>
#include <vector>

#include "fwd.hh"
#include "logical_time.hh"

namespace reactor {

class Scheduler {
 public:
  using EventMap = std::map<BaseAction*, std::function<void(void)>>;

 private:
  const bool using_workers;
  LogicalTime _logical_time{};

  Environment* _environment;
  std::vector<std::thread> worker_threads;

  std::mutex m_schedule;
  std::unique_lock<std::mutex> schedule_lock{m_schedule, std::defer_lock};
  std::condition_variable cv_schedule;

  std::mutex m_event_queue;
  std::map<Tag, std::unique_ptr<EventMap>> event_queue;
  std::vector<BasePort*> set_ports;

  std::mutex m_reaction_queue;
  std::vector<std::vector<Reaction*>> reaction_queue;
  unsigned reaction_queue_pos{std::numeric_limits<unsigned>::max()};

  std::mutex m_ready_reactions;
  std::vector<Reaction*> ready_reactions;

  std::counting_semaphore<1024> sem_running_workers{1};
  std::atomic<unsigned> running_workers{1};

  void work(unsigned id);
  void process_ready_reactions(unsigned id);
  void schedule_ready_reactions(unsigned id);

  void next();

  void set_port_helper(BasePort* p);

  std::atomic<bool> _stop{false};
  std::atomic<bool> terminate_workers{false};
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
};

}  // namespace reactor
