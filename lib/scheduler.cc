/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/scheduler.hh"
#include "dear/logging.hh"

#include <vector>

namespace dear {

void Scheduler::work(unsigned id) {
  log::Debug() << "Starting worker " << id;

  while (true) {
    std::unique_lock<std::mutex> lock(m_work_queue);
    cv_workers.wait(lock, [this]() { return this->work_queue.size() > 0; });

    auto work = std::move(work_queue.back());
    work_queue.pop_back();
    lock.unlock();

    // do the work
    (*work)();
  }
}

void Scheduler::start() {
  log::Debug() << "Starting the scheduler...";

  // start worker threads
  for (unsigned i; i < num_workers; i++) {
    worker_threads.emplace_back([this, i]() { this->work(i); });
  }

  // enqueue some work
  std::vector<std::future<void>> futures(10);
  {
    std::lock_guard<std::mutex> lg(m_work_queue);
    for (int i = 0; i < 10; i++) {
      auto work = std::make_unique<WorkItem>(
          [i]() { log::Info() << "working on " << i; });
      futures[i] = work->get_future();
      work_queue.emplace_back(std::move(work));
    }
  }
  cv_workers.notify_all();

  // wait for the tasks to finish
  for (auto& f : futures) {
    f.get();
  }
}

Scheduler::~Scheduler() {
  // join all worker threads
  for (auto& t : worker_threads) {
    t.join();
  }
}

}  // namespace dear
