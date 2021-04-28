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
#include "reactor-cpp/trace.hh"

namespace reactor {

void Scheduler::work(unsigned id) {
  log::Debug() << "(Worker " << id << ") "
               << "Starting";

  while (!terminate_workers.load(std::memory_order_acquire)) {
    // process ready reactions as long as there are any
    process_ready_reactions(id);

    {
      auto lg = using_workers ? std::unique_lock<std::mutex>(m_running_workers)
                              : std::unique_lock<std::mutex>();

      // let all workers except the last one sleep until new reactions are added
      // to the ready queue
      if (using_workers) {
        if (running_workers > 1) {
          // wait for reactions to become ready for execution, or for a
          // terminate signal
          wait_for_ready_reactions(id, lg);
          continue;  // start from the top after waking up
        }
        ASSERT(running_workers == 1);
        log::Debug() << "(Worker " << id << ") "
                     << "I am the last active worker.";
      }

      // Reaching this point means that the current worker thread is the last
      // running worker. Since holding the m_running_workers mutex guarantees
      // that no other worker is running, we can safely work with all the data
      // structures without acquiring additional mutexes.

      // Have we finished iterating over the reaction queue?
      if (reaction_queue_pos < reaction_queue.size()) {
        // No -> continue iterating
        log::Debug() << "(Worker " << id << ") "
                     << "Scanning the reaction queue for ready reactions";

        // continue the actual iteration
        while (reaction_queue_pos < reaction_queue.size() &&
               reaction_queue[reaction_queue_pos].empty()) {
          reaction_queue_pos++;
        }

        if (reaction_queue_pos < reaction_queue.size()) {
          auto& reactions = reaction_queue[reaction_queue_pos];

          // any ready reactions of current priority?
          if (!reactions.empty()) {
            log::Debug() << "(Worker " << id << ") "
                         << "Process reactions of priority "
                         << reaction_queue_pos;

            // Make sure that any reaction is only executed once even if it
            // was triggered multiple times.
            std::sort(reactions.begin(), reactions.end());
            reactions.erase(std::unique(reactions.begin(), reactions.end()),
                            reactions.end());

            // place all ready reactions on the ready queue
            for (auto r : reactions) {
              log::Debug() << "(Worker " << id << ") "
                           << "Reaction " << r->fqn()
                           << " is ready for execution";
              tracepoint(reactor_cpp, trigger_reaction, r->container()->fqn(),
                         r->name(), _logical_time);
              ready_reactions.push_back(r);
            }

            reactions.clear();
          }
        }

        if (ready_reactions.size() == 1) {
          // if there is only one reaction, then this worker processes it
          // directly
          continue;
        } else if (ready_reactions.size() > 1) {
          // notify other workers if there is more than one ready reaction
          if (using_workers) {
            lg.unlock();
            cv_ready_reactions.notify_all();
          }
          continue;
        }
      }
    }  // m_running_workers

    // if we reach this point, all reactions in the ready queue where processed
    // and we need to call next() or terminate the execution
    if (continue_execution) {
      log::Debug() << "(Worker " << id << ") "
                   << "call next()";
      next();
      reaction_queue_pos = 0;
    } else {
      terminate_workers.store(true, std::memory_order_release);
      cv_ready_reactions.notify_all();
    }
  }

  log::Debug() << "Stopping worker " << id;
}

void Scheduler::process_ready_reactions(unsigned id) {
  auto lg = using_workers ? std::unique_lock<std::mutex>(m_ready_reactions)
                          : std::unique_lock<std::mutex>();

  // process ready reactions as long as there are any
  while (!ready_reactions.empty()) {
    // Check if there are reactions to be executed.
    // If there is a ready reaction, remove it from the ready queue and add
    // it to the list of reactions that are currently executing.
    auto reaction = ready_reactions.back();
    ready_reactions.pop_back();

    if (using_workers)
      lg.unlock();

    // execute the reaction
    log::Debug() << "(Worker " << id << ") "
                 << "execute reaction " << reaction->fqn();
    tracepoint(reactor_cpp, reaction_execution_starts, id, reaction->fqn());
    reaction->trigger();
    tracepoint(reactor_cpp, reaction_execution_finishes, id, reaction->fqn());

    if (using_workers)
      lg.lock();
  }
}

void Scheduler::wait_for_ready_reactions(unsigned id,
                                         std::unique_lock<std::mutex>& lock) {
  log::Debug() << "(Worker " << id << ") "
               << "wait for ready reactions";
  running_workers--;
  cv_ready_reactions.wait(lock, [this]() {
    return !this->ready_reactions.empty() || terminate_workers;
  });
  running_workers++;
}

void Scheduler::start() {
  log::Debug() << "Starting the scheduler...";

  // initialize the reaction queue
  reaction_queue.resize(_environment->max_reaction_index() + 1);

  running_workers = _environment->num_workers();
  // start worker threads
  for (unsigned i = 1; i < _environment->num_workers() + 1; i++) {
    worker_threads.emplace_back([this, i]() { this->work(i); });
  }

  // join all worker threads
  for (auto& t : worker_threads) {
    t.join();
  }
}

void Scheduler::next() {
  static std::unique_ptr<EventMap> events{nullptr};

  // clean up before scheduling any new events
  if (events != nullptr) {
    // cleanup all triggered actions
    for (auto& kv : *events) {
      kv.first->cleanup();
    }
    events = nullptr;

    // cleanup all set ports
    for (auto p : set_ports) {
      p->cleanup();
    }
    set_ports.clear();
  }

  {
    std::unique_lock<std::mutex> lock{m_schedule};

    // shutdown if there are no more events in the queue
    if (event_queue.empty() && !_stop) {
      if (_environment->run_forever()) {
        // wait for a new asynchronous event
        cv_schedule.wait(lock,
                         [this]() { return !event_queue.empty() || _stop; });
      } else {
        log::Debug() << "No more events in queue. -> Terminate!";
        _environment->sync_shutdown();
      }
    }

    while (events == nullptr) {
      if (_stop) {
        continue_execution = false;
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
          return;
        }
      } else {
        // collect events of the next tag
        auto t_next = event_queue.begin()->first;

        // synchronize with physical time if not in fast forward mode
        if (!_environment->fast_fwd_execution()) {
          // keep track of the current physical time in a static variable
          static auto physical_time = TimePoint::min();

          // If physical time is smaller than the next logical time point,
          // then update the physical time. This step is small optimization to
          // avoid calling get_physical_time() in every iteration as this
          // would add a significant overhead.
          if (physical_time < t_next.time_point())
            physical_time = get_physical_time();

          // If physical time is still smaller than the next logical time
          // point, then wait until the next tag or until a new event is
          // inserted asynchronously into the queue
          if (physical_time < t_next.time_point()) {
            auto status = cv_schedule.wait_until(lock, t_next.time_point());
            // Start over if the event queue was modified
            if (status == std::cv_status::no_timeout) {
              continue;
            } else {
              // update physical time and continue otherwise
              physical_time = t_next.time_point();
            }
          }
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

  for (auto& kv : *events) {
    for (auto n : kv.first->triggers()) {
      // There is no need to acquire the mutex. At this point the scheduler
      // should be the only thread accessing the reaction queue as none of the
      // workers are running
      reaction_queue[n->index()].push_back(n);
    }
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

    tracepoint(reactor_cpp, schedule_action, action->container()->fqn(),
               action->name(), tag);

    // create a new event map or retrieve the existing one
    auto emplace_result =
        event_queue.try_emplace(tag, std::make_unique<EventMap>());
    auto& event_map = *emplace_result.first->second;

    // insert the new event
    event_map[action] = setup;
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
  auto lg = using_workers ? std::unique_lock<std::mutex>(m_reaction_queue)
                          : std::unique_lock<std::mutex>();
  // We do not check here if p is already in the list. This means clean()
  // could be called multiple times for a single port. However, calling
  // clean() multiple time is not harmful and more efficient then checking if
  // the port is already in the list.
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
