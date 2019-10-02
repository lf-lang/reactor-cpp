/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include "dear/fwd.hh"
#include "dear/logical_time.hh"

#include <condition_variable>
#include <functional>
#include <future>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

namespace dear {

class Scheduler {
 public:
  using WorkItem = std::packaged_task<void(void)>;
  using WorkPtr = std::unique_ptr<WorkItem>;
  using EventMap = std::map<BaseAction*, std::function<void(void)>>;

 private:
  bool terminate{false};
  LogicalTime _logical_time{};

  std::vector<WorkPtr> work_queue;
  std::mutex m_work_queue;
  std::condition_variable cv_workers;

  const unsigned num_workers;
  std::vector<std::thread> worker_threads;

  std::mutex m_post_handlers;
  std::vector<std::function<void(void)>> post_handlers;

  std::mutex m_event_queue;
  std::map<Tag, std::unique_ptr<EventMap>> event_queue;

  std::map<unsigned, std::vector<Reaction*>> ready_reactions;

  void work(unsigned id);

  void next();

  Tag next_tag();

  void wait_for_physical_time(const Tag& tag);

 public:
  Scheduler(unsigned num_workers) : num_workers(num_workers) {}
  ~Scheduler();

  void schedule(const Tag& tag,
                BaseAction* action,
                std::function<void(void)> pre_handler);

  const LogicalTime& logical_time() const { return _logical_time; }

  void start();
};

}  // namespace dear
