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
#include <mutex>
#include <thread>
#include <vector>

namespace dear {

class Scheduler {
 public:
  using WorkItem = std::packaged_task<void(void)>;
  using WorkPtr = std::unique_ptr<WorkItem>;

 private:
  std::vector<WorkPtr> work_queue;
  std::mutex m_work_queue;
  std::condition_variable cv_workers;

  const unsigned num_workers;
  std::vector<std::thread> worker_threads;

  void work(unsigned id);

 public:
  Scheduler(unsigned num_workers) : num_workers(num_workers) {}
  ~Scheduler();

  void start();
};

}  // namespace dear
