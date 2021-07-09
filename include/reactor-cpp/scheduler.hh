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

// forward declarations
class Scheduler;
class Worker;

class Worker {
 public:
  Scheduler& scheduler;
  const unsigned id{0};
  std::thread thread;

  static thread_local const Worker* current_worker;

  void work() const;
  void execute_reaction(Reaction* reaction) const;

 public:
  Worker(Scheduler& scheduler, unsigned id)
      : scheduler{scheduler}, id{id}, thread{} {}
  Worker(Worker&& w);
  Worker(const Worker&) = delete;

  void start_thread() { thread = std::thread(&Worker::work, this); }
  void join_thread() { thread.join(); }

  static unsigned current_worker_id() { return current_worker->id; }
};

class ReadyQueue {
 private:
  std::vector<Reaction*> queue{};
  std::atomic<std::ptrdiff_t> size{0};
  BaseSemaphore sem{0};
  std::ptrdiff_t waiting_workers{0};
  const unsigned num_workers;

 public:
  ReadyQueue(unsigned num_workers) : num_workers(num_workers) {}

  /**
   * Retrieve a ready reaction from the queue.
   *
   * This method may be called concurrently. In case the queue is empty, the
   * method blocks and waits until a ready reaction becomes available.
   */
  Reaction* pop();

  /**
   * Fill the queue up with ready reactions.
   *
   * This method assumes that the internal queue is empty. It moves all
   * reactions from the provided `ready_reactions` vector to the internal queue,
   * leaving `ready_reactions` empty.
   *
   * Note that this method is not thread-safe. The caller needs to ensure that
   * no other thread will try to read from the queue during this operation.
   */
  void fill_up(std::vector<Reaction*>& ready_reactions);
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
  std::map<Tag, EventMap> event_queue;

  std::vector<std::vector<BasePort*>> set_ports;
  std::vector<std::vector<Reaction*>> triggered_reactions;

  std::vector<std::vector<Reaction*>> reaction_queue;
  unsigned reaction_queue_pos{std::numeric_limits<unsigned>::max()};

  ReadyQueue ready_queue;
  std::atomic<std::ptrdiff_t> reactions_to_process{0};

  void schedule();
  bool schedule_ready_reactions();

  void next();

  void terminate_all_workers();

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
