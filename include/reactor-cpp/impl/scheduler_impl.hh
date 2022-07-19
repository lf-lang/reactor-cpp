/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_IMPL_SCHEDULER_IMPL_HH
#define REACTOR_CPP_IMPL_SCHEDULER_IMPL_HH

#include "reactor-cpp/trace.hh"
#include <reactor-cpp/logging.hh>
#include <reactor-cpp/reaction.hh>

namespace reactor {

template <class SchedulingPolicy>
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
thread_local const Worker<SchedulingPolicy>* Worker<SchedulingPolicy>::current_worker = nullptr;

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
  current_worker = this;

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

} // namespace reactor

#endif // REACTOR_CPP_IMPL_SCHEDULER_IMPL_HH
