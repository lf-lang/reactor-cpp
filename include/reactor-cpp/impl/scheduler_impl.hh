/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_IMPL_SCHEDULER_IMPL_HH
#define REACTOR_CPP_IMPL_SCHEDULER_IMPL_HH

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/fwd.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/trace.hh"

#include <cstddef>

namespace reactor {

template <class SchedulingPolicy>
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
thread_local std::size_t Worker<SchedulingPolicy>::current_worker_id_{0};

template <class SchedulingPolicy> auto Worker<SchedulingPolicy>::current_worker_id() -> unsigned {
  return current_worker_id_;
}

template <class SchedulingPolicy>
Worker<SchedulingPolicy>::Worker(Worker<SchedulingPolicy>&& worker) // NOLINT(performance-noexcept-move-constructor)
    : policy_{worker.policy_}
    , identity_{worker.identity_} {
  // Need to provide the move constructor in order to organize workers in a
  // std::vector. However, moving is not save if the thread is already running,
  // thus we throw an exception here if the worker is moved but the
  // internal thread is already running.

  if (worker.thread_.joinable()) {
    throw std::runtime_error{"Running workers cannot be moved!"};
  }
}

template <class SchedulingPolicy> void Worker<SchedulingPolicy>::work() const {
  // initialize the current worker thread local variable
  current_worker_id_ = identity_;

  log::Debug() << "(Worker " << identity_ << ") Starting";

  policy_.worker_function(*this);

  log::Debug() << "(Worker " << identity_ << ") terminates";
}

template <class SchedulingPolicy> void Worker<SchedulingPolicy>::execute_reaction(Reaction* reaction) const {
  log::Debug() << "(Worker " << identity_ << ") "
               << "execute reaction " << reaction->fqn();

  tracepoint(reactor_cpp, reaction_execution_starts, id, reaction->fqn(), scheduler.logical_time());
  reaction->trigger();
  tracepoint(reactor_cpp, reaction_execution_finishes, id, reaction->fqn(), scheduler.logical_time());
}

template <class SchedulingPolicy>
Scheduler<SchedulingPolicy>::Scheduler(Environment* env)
    : BaseScheduler(env)
    , policy_(*this, *env) {}

template <class SchedulingPolicy> void Scheduler<SchedulingPolicy>::start() {
  log::Debug() << "Starting the scheduler...";

  auto num_workers = environment_->num_workers();
  // initialize the reaction queue, set ports vector, and triggered reactions
  // vector
  set_ports_.resize(num_workers);
  policy_.init();

  // Initialize and start the workers. By resizing the workers vector first,
  // we make sure that there is sufficient space for all the workers and non of
  // them needs to be moved. This is important because a running worker may not
  // be moved.
  workers_.reserve(num_workers);
  for (unsigned i = 0; i < num_workers; i++) {
    workers_.emplace_back(policy_.create_worker());
    workers_.back().start();
  }

  // join all worker threads
  for (auto& worker : workers_) {
    worker.join();
  }
}

// FIXME: Reduce complexity of this function
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
template <class SchedulingPolicy> auto Scheduler<SchedulingPolicy>::next() -> bool {
  static EventMap events{};
  bool continue_execution{true};

  // clean up before scheduling any new events
  if (!events.empty()) {
    // cleanup all triggered actions
    for (auto& vec_ports : events) {
      vec_ports.first->cleanup();
    }
    // cleanup all set ports
    for (auto& vec_ports : set_ports_) {
      for (auto& port : vec_ports) {
        port->cleanup();
      }
      vec_ports.clear();
    }
    events.clear();
  }

  {
    std::unique_lock<std::mutex> lock{scheduling_mutex_};

    // shutdown if there are no more events in the queue
    if (event_queue_.empty() && !stop_) {
      if (environment_->run_forever()) {
        // wait for a new asynchronous event
        cv_schedule_.wait(lock, [this]() { return !event_queue_.empty() || stop_; });
      } else {
        log::Debug() << "No more events in queue_. -> Terminate!";
        environment_->sync_shutdown();
      }
    }

    while (events.empty()) {
      if (stop_) {
        continue_execution = false;
        log::Debug() << "Shutting down the scheduler";
        Tag t_next = Tag::from_logical_time(logical_time_).delay();
        if (t_next == event_queue_.begin()->first) {
          log::Debug() << "Schedule the last round of reactions including all "
                          "termination reactions";
          events = std::move(event_queue_.begin()->second);
          event_queue_.erase(event_queue_.begin());
          log::Debug() << "advance logical time to tag [" << t_next.time_point() << ", " << t_next.micro_step() << "]";
          logical_time_.advance_to(t_next);
        } else {
          return continue_execution;
        }
      } else {
        // collect events of the next tag
        auto t_next = event_queue_.begin()->first;

        // synchronize with physical time if not in fast forward mode
        if (!environment_->fast_fwd_execution()) {
          // keep track of the current physical time in a static variable
          static auto physical_time = TimePoint::min();

          // If physical time is smaller than the next logical time point,
          // then update the physical time. This step is small optimization to
          // avoid calling get_physical_time() in every iteration as this
          // would add a significant overhead.
          if (physical_time < t_next.time_point()) {
            physical_time = get_physical_time();
          }

          // If physical time is still smaller than the next logical time
          // point, then wait until the next tag or until a new event is
          // inserted asynchronously into the queue
          if (physical_time < t_next.time_point()) {
            auto status = cv_schedule_.wait_until(lock, t_next.time_point());
            // Start over if the event queue was modified
            if (status == std::cv_status::no_timeout) {
              continue;
            }
            // update physical time and continue otherwise
            physical_time = t_next.time_point();
          }
        }

        // retrieve all events with tag equal to current logical time from the
        // queue
        events = std::move(event_queue_.begin()->second);
        event_queue_.erase(event_queue_.begin());

        // advance logical time
        log::Debug() << "advance logical time to tag [" << t_next.time_point() << ", " << t_next.micro_step() << "]";
        logical_time_.advance_to(t_next);
      }
    }
  } // mutex schedule_

  // execute all setup functions; this sets the values of the corresponding
  // actions
  for (auto& vec_reactor : events) {
    auto& setup = vec_reactor.second;
    if (setup != nullptr) {
      setup();
    }
  }

  log::Debug() << "events: " << events.size();
  for (auto& vec_reactor : events) {
    log::Debug() << "Action " << vec_reactor.first->fqn();
    for (auto* reaction : vec_reactor.first->triggers()) {
      // There is no need to acquire the mutex. At this point the scheduler
      // should be the only thread accessing the reaction queue as none of the
      // workers_ are running
      log::Debug() << "(Scheduler) trigger reaction " << reaction->fqn() << " with index " << reaction->index();
      policy_.trigger_reaction_from_next(reaction);
    }
  }

  return continue_execution;
}

template <class SchedulingPolicy> void Scheduler<SchedulingPolicy>::set_port(BasePort* port) {
  log::Debug() << "Set port " << port->fqn();

  // We do not check here if port is already in the list. This means clean()
  // could be called multiple times for a single port. However, calling
  // clean() multiple time is not harmful and more efficient then checking if
  set_ports_[Worker<SchedulingPolicy>::current_worker_id()].push_back(port);

  // recursively search for triggered reactions
  set_port_helper(port);
}

template <class SchedulingPolicy> void Scheduler<SchedulingPolicy>::set_port_helper(BasePort* port) {
  for (auto* reaction : port->triggers()) {
    policy_.trigger_reaction_from_set_port(reaction);
  }
  for (auto* binding : port->outward_bindings()) {
    set_port_helper(binding);
  }
}

template <class SchedulingPolicy> void Scheduler<SchedulingPolicy>::stop() {
  stop_ = true;
  cv_schedule_.notify_one();
}

} // namespace reactor

#endif // REACTOR_CPP_IMPL_SCHEDULER_IMPL_HH
