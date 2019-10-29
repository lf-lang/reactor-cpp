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
#include "dear/port.hh"
#include "dear/reaction.hh"

namespace dear {

void Scheduler::work(unsigned id) {
  log::Debug() << "Starting worker " << id;

  while (true) {
    std::unique_lock<std::mutex> lock(m_reaction_queue);
    cv_ready_reactions.wait(
        lock, [this]() { return !this->ready_reactions.empty() || terminate; });
    if (terminate) {
      lock.unlock();
      break;
    }

    auto reaction = ready_reactions.back();
    ready_reactions.pop_back();
    executing_reactions.insert(reaction);
    lock.unlock();

    log::Debug() << "Execute reaction " << reaction->fqn();

    // do the work
    auto body{reaction->body()};
    body();

    lock.lock();
    executing_reactions.erase(reaction);
    lock.unlock();
    cv_done_reactions.notify_one();
  }

  log::Debug() << "Stopping worker " << id;
}

void Scheduler::start() {
  log::Debug() << "Starting the scheduler...";

  // start worker threads
  for (unsigned i = 0; i < num_workers; i++) {
    worker_threads.emplace_back([this, i]() { this->work(i); });
  }

  while (event_queue.size() > 0) {
    next();
  }

  log::Debug() << "No more events in queue. -> Terminate!";

  terminate = true;
  cv_ready_reactions.notify_all();

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
  log::Debug() << "advance logical time to tag [" << t_next.time() << ", "
               << t_next.micro_step() << "]";
  _logical_time.advance_to(t_next);

  // execute all setup functions; this sets the values of the corresponding
  // actions
  for (auto& kv : *events) {
    auto& setup = kv.second;
    if (setup != nullptr) {
      setup();
    }
  }

  std::vector<std::future<void>> futures;
  for (auto& kv : *events) {
    for (auto n : kv.first->triggers()) {
      // There is no need to acquire the mutex. At this point the scheduler
      // should be the only thread accessing the reaction queue as none of the
      // workers are running
      reaction_queue[_environment->get_index(n)].insert(n);
    }
  }

  // process all reactions in the queue
  while (!reaction_queue.empty()) {
    // only process reaction with highest priority
    auto& reactions = reaction_queue.begin()->second;
    log::Debug() << "Process reactions of priority "
                 << reaction_queue.begin()->first;

    // we need to acquire the mutex as workers might be running
    std::unique_lock<std::mutex> lock(m_reaction_queue);
    while (!reactions.empty() || !ready_reactions.empty() ||
           !executing_reactions.empty()) {
      // if there are reactions in the queue, mark them as ready to execute
      if (!reactions.empty()) {
        for (auto r : reactions) {
          log::Debug() << "Schedule reaction " << r->fqn();
          ready_reactions.push_back(r);
        }
        reactions.clear();
        // notify workers about ready reactions
        cv_ready_reactions.notify_all();
      }
      log::Debug() << "Waiting for workers ...";
      cv_done_reactions.wait(lock);
    }
    reaction_queue.erase(reaction_queue.begin());
    lock.unlock();
  }

  // cleanup all triggered actions
  for (auto& kv : *events) {
    kv.first->cleanup();
  }

  // cleanup all set ports
  for (auto p : set_ports) {
    p->cleanup();
  }
  set_ports.clear();
}

void Scheduler::wait_for_physical_time(const Tag& tag) {
  if (tag.time() > get_physical_timepoint()) {
    wait_until_physical_timepoint(tag.time());
  }
}

Scheduler::~Scheduler() {}

void Scheduler::schedule(const Tag& tag,
                         BaseAction* action,
                         std::function<void(void)> setup) {
  ASSERT(_logical_time < tag);
  // TODO verify that the action is indeed allowed to be scheduled by the
  // current reaction
  log::Debug() << "Schedule action " << action->fqn() << " with tag ["
               << tag.time() << ", " << tag.micro_step() << "]";

  std::lock_guard<std::mutex> lg(m_event_queue);
  if (event_queue.find(tag) == event_queue.end())
    event_queue.emplace(tag, std::make_unique<EventMap>());

  (*event_queue[tag])[action] = setup;
}

void Scheduler::set_port(BasePort* p) {
  log::Debug() << "Set port " << p->fqn();
  std::lock_guard<std::mutex> lg(m_reaction_queue);
  set_ports.insert(p);
  // recursively search for triggered reactions
  set_port_helper(p);
}

void Scheduler::set_port_helper(BasePort* p) {
  ASSERT(!(p->has_outward_bindings() && !p->triggers().empty()));
  if (p->has_outward_bindings()) {
    for (auto binding : p->outward_bindings()) {
      set_port_helper(binding);
    }
  } else {
    for (auto n : p->triggers()) {
      reaction_queue[_environment->get_index(n)].insert(n);
    }
  }
}

}  // namespace dear
