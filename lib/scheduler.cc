/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/scheduler.hh"
#include "dear/action.hh"
#include "dear/assert.hh"
#include "dear/logging.hh"
#include "dear/reaction.hh"

namespace dear {

void Scheduler::work(unsigned id) {
  log::Debug() << "Starting worker " << id;

  while (true) {
    std::unique_lock<std::mutex> lock(m_work_queue);
    cv_workers.wait(
        lock, [this]() { return this->work_queue.size() > 0 || terminate; });
    if (terminate) {
      lock.unlock();
      break;
    }

    auto work = std::move(work_queue.back());
    work_queue.pop_back();
    lock.unlock();

    // do the work
    (*work)();
  }

  log::Debug() << "Stopping worker " << id;
}

void Scheduler::start() {
  log::Debug() << "Starting the scheduler...";

  // start worker threads
  for (unsigned i; i < num_workers; i++) {
    worker_threads.emplace_back([this, i]() { this->work(i); });
  }

  while (event_queue.size() > 0) {
    next();
  }

  log::Debug() << "No more events in queue. -> Terminate!";

  terminate = true;
  cv_workers.notify_all();

  // join all worker threads
  for (auto& t : worker_threads) {
    t.join();
  }
}

void Scheduler::next() {
  // We can access event_queue here without acquiring the mutex as all
  // other threads should be waiting

  ASSERT(event_queue.size() > 0);

  // extract all events with the next tag from the queue
  auto& t_next = event_queue.begin()->first;
  auto events = std::move(event_queue.begin()->second);
  event_queue.erase(event_queue.begin());

  // align with the physical clock
  wait_for_physical_time(t_next);

  // advance logical time
  _logical_time.advance_to(t_next);

  // execute all pre_handlers; this sets the values of the corresponding
  // actions
  for (auto& kv : *events) {
    auto& pre_handler = kv.second;
    if (pre_handler != nullptr) {
      pre_handler();
    }
  }

  std::vector<std::future<void>> futures;
  for (auto& kv : *events) {
    for (auto n : kv.first->triggers()) {
      auto task = std::make_unique<WorkItem>(n->body());
      futures.push_back(task->get_future());
      // no need to acquire the mutex as there shouldn't be any
      // workers running
      work_queue.push_back(std::move(task));
    }
  }
  // notify workers to start processing enqueued tasks
  cv_workers.notify_all();

  // wait for all tasks to finish
  for (auto& f : futures) {
    f.get();
  }
}

void Scheduler::wait_for_physical_time(const Tag& tag) {
  if (tag.time() > get_physical_timepoint()) {
    wait_until_physical_timepoint(tag.time());
  }
}

Scheduler::~Scheduler() {}

void Scheduler::schedule(const Tag& tag,
                         BaseAction* action,
                         std::function<void(void)> pre_handler) {
  ASSERT(_logical_time < tag);
  // TODO verify that the action is indeed allowed to be scheduled by the
  // current reaction

  std::lock_guard<std::mutex> lg(m_event_queue);
  if (event_queue.find(tag) == event_queue.end())
    event_queue.emplace(tag, std::make_unique<EventMap>());

  (*event_queue[tag])[action] = pre_handler;
}

}  // namespace dear
