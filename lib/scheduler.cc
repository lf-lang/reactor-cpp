/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/scheduler.hh"

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"

namespace reactor {

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
    reaction->trigger();

    lock.lock();
    executing_reactions.erase(reaction);
    lock.unlock();
    cv_done_reactions.notify_one();
  }

  log::Debug() << "Stopping worker " << id;
}

void Scheduler::start() {
  log::Debug() << "Starting the scheduler...";

  // initialize the reaction queue
  reaction_queue.resize(_environment->max_reaction_index() + 1);

  if (using_workers) {
    // start worker threads
    for (unsigned i = 0; i < _environment->num_workers(); i++) {
      worker_threads.emplace_back([this, i]() { this->work(i); });
    }
  }

  while (next()) {
  }

  {
    std::lock_guard<std::mutex> lg(m_reaction_queue);
    terminate = true;
  }
  cv_ready_reactions.notify_all();

  // join all worker threads
  for (auto& t : worker_threads) {
    t.join();
  }
}

bool Scheduler::next() {
  std::unique_ptr<EventMap> events{nullptr};
  bool run_again = true;

  {
    std::unique_lock<std::mutex> lock{m_schedule};

    // shutdown if there are no more events in the queue
    if (event_queue.empty() && !_stop) {
      if (_environment->run_forever()) {
        // wait for a new asynchronous event
        cv_schedule.wait(lock, [this]() { return !event_queue.empty(); });
      } else {
        log::Debug() << "No more events in queue. -> Terminate!";
        _environment->sync_shutdown();

        // The shutdown call might schedule shutdown reactions. If non was
        // scheduled, we simply return.
        if (event_queue.empty()) {
          return false;
        }
      }
    }

    if (_stop) {
      run_again = false;
      log::Debug() << "Shutting down the scheduler";
      Tag t_next = Tag::from_logical_time(_logical_time).delay();
      if (t_next == event_queue.begin()->first) {
        log::Debug() << "Schedule the last round of reactions including all "
                        "termination reactions";
        events = std::move(event_queue.begin()->second);
        event_queue.erase(event_queue.begin());
        log::Debug() << "advance logical time to tag [" << t_next.time_point()
                     << ", " << t_next.micro_step() << "]";
        _logical_time.advance_to(t_next);
      } else {
        return false;
      }
    } else {
      // collect events of the next tag
      auto t_next = event_queue.begin()->first;

      // synchronize with physical time if not in fast forward mode
      if (!_environment->fast_fwd_execution()) {
        // wait until the next tag or until a new event is inserted
        // asynchronously into the queue
        auto status = cv_schedule.wait_until(lock, t_next.time_point());
        // Start over if the event queue was modified
        if (status == std::cv_status::no_timeout) {
          return true;
        }
        // continue otherwise
      }

      // retrieve all events with tag equal to current logical time from the
      // queue
      events = std::move(event_queue.begin()->second);
      event_queue.erase(event_queue.begin());

      // advance logical time
      log::Debug() << "advance logical time to tag [" << t_next.time_point()
                   << ", " << t_next.micro_step() << "]";
      _logical_time.advance_to(t_next);
    }
  }  // mutex m_schedule

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
      reaction_queue[n->index()].push_back(n);
    }
  }

  // process all reactions in the queue in order of there index
  for (unsigned i = 0; i < reaction_queue.size(); i++) {
    auto& reactions = reaction_queue[i];

    if (!reactions.empty()) {
      log::Debug() << "Process reactions of priority " << i;

      // Make sure that any reaction is only executed once even if it was
      // triggered multiple times.
      std::sort(reactions.begin(), reactions.end());
      reactions.erase(std::unique(reactions.begin(), reactions.end()),
                      reactions.end());

      if (using_workers && reactions.size() > 1) {
        dispatch_reactions_to_workers(reactions);
      } else {
        execute_reactions_inline(reactions);
      }
      reactions.clear();
    }
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

  return run_again;
}  // namespace reactor

void Scheduler::dispatch_reactions_to_workers(
    const std::vector<Reaction*>& reactions) {
  // dispatch all ready reactions
  std::unique_lock<std::mutex> lock(m_reaction_queue);
  for (auto r : reactions) {
    log::Debug() << "Schedule reaction " << r->fqn();
    ready_reactions.push_back(r);
  }
  lock.unlock();

  // notify workers about ready reactions
  cv_ready_reactions.notify_all();

  // we need to acquire the mutex now as workers are running
  lock.lock();
  while (!ready_reactions.empty() || !executing_reactions.empty()) {
    log::Debug() << "Waiting for workers ...";
    cv_done_reactions.wait(lock);
  }
  lock.unlock();
}

void Scheduler::execute_reactions_inline(
    const std::vector<Reaction*>& reactions) {
  for (auto r : reactions) {
    log::Debug() << "Execute reaction " << r->fqn();
    r->trigger();
  }
}

Scheduler::Scheduler(Environment* env)
    : using_workers(env->num_workers() > 1), _environment(env) {}

Scheduler::~Scheduler() {}

void Scheduler::schedule_sync(const Tag& tag,
                              BaseAction* action,
                              std::function<void(void)> setup) {
  ASSERT(_logical_time < tag);
  // TODO verify that the action is indeed allowed to be scheduled by the
  // current reaction
  log::Debug() << "Schedule action " << action->fqn()
               << (action->is_logical() ? " synchronously "
                                        : " asynchronously ")
               << " with tag [" << tag.time_point() << ", " << tag.micro_step()
               << "]";
  {
    auto lg = using_workers ? std::unique_lock<std::mutex>(m_event_queue)
                            : std::unique_lock<std::mutex>();
    if (event_queue.find(tag) == event_queue.end())
      event_queue.emplace(tag, std::make_unique<EventMap>());

    (*event_queue[tag])[action] = setup;
  }
}

void Scheduler::schedule_async(const Tag& tag,
                               BaseAction* action,
                               std::function<void(void)> setup) {
  std::lock_guard<std::mutex> lg(m_schedule);
  schedule_sync(tag, action, setup);
  cv_schedule.notify_one();
}

void Scheduler::set_port(BasePort* p) {
  log::Debug() << "Set port " << p->fqn();
  auto lg = using_workers ? std::unique_lock<std::mutex>(m_event_queue)
                          : std::unique_lock<std::mutex>();
  // We do not check here if p is already in the list. This means clean()
  // could be called multiple times for a single port. However, calling clean()
  // multiple time is not harmful and more efficient then checking if the
  // port is already in the list.
  set_ports.push_back(p);
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
      reaction_queue[n->index()].push_back(n);
    }
  }
}

void Scheduler::stop() {
  _stop = true;
  cv_schedule.notify_one();
}

}  // namespace reactor
