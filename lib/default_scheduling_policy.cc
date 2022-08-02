/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/default_scheduling_policy.hh"

#include "reactor-cpp/environment.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/scheduler.hh"
#include "reactor-cpp/trace.hh"

namespace reactor {

DefaultSchedulingPolicy::DefaultSchedulingPolicy(Scheduler<DefaultSchedulingPolicy>& scheduler, Environment& env)
    : scheduler_(scheduler)
    , environment_(env)
    , ready_queue_(env.num_workers()) {}

void DefaultSchedulingPolicy::init() {
  reaction_queue_.resize(environment_.max_reaction_index() + 1);
  triggered_reactions_.resize(environment_.num_workers());
}

void DefaultSchedulingPolicy::worker_function(const Worker<DefaultSchedulingPolicy>& worker) {
  if (worker.id() == 0) {
    log::Debug() << "(Worker 0) do the initial scheduling";
    schedule();
  }

  while (true) {
    // wait for a ready reaction
    auto* reaction = ready_queue_.pop();

    // receiving a nullptr indicates that the worker should terminate
    if (reaction == nullptr) {
      break;
    }

    // execute the reaction
    worker.execute_reaction(reaction);

    // was this the very last reaction?
    if (reactions_to_process_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      // Yes, then schedule. The atomic decrement above ensures that only one
      // thread enters this block.
      schedule();
    }
    // continue otherwise
  }
}

auto DefaultSchedulingPolicy::create_worker() -> Worker<DefaultSchedulingPolicy> { return {*this, identity_counter++}; }

void DefaultSchedulingPolicy::schedule() noexcept {
  bool found_ready_reactions = schedule_ready_reactions();

  while (!found_ready_reactions) {
    log::Debug() << "(DefaultSchedulingPolicy) call next()";
    continue_execution_ = scheduler_.next();
    reaction_queue_pos_ = 0;

    found_ready_reactions = schedule_ready_reactions();

    if (!continue_execution_ && !found_ready_reactions) {
      // let all workers know that they should terminate
      terminate_all_workers();
      break;
    }
  }
}

auto DefaultSchedulingPolicy::schedule_ready_reactions() -> bool {
  // insert any triggered reactions_ into the reaction queue
  for (auto& vec_reaction : triggered_reactions_) {
    for (auto* reaction : vec_reaction) {
      reaction_queue_[reaction->index()].push_back(reaction);
    }
    vec_reaction.clear();
  }

  log::Debug() << "(DefaultSchedulingPolicy) Scanning the reaction queue for ready reactions";

  // continue iterating over the reaction queue
  for (; reaction_queue_pos_ < reaction_queue_.size(); reaction_queue_pos_++) {
    auto& reactions = reaction_queue_[reaction_queue_pos_];

    // any ready reactions of current priority?
    if (!reactions.empty()) {
      log::Debug() << "(DefaultSchedulingPolicy) Process reactions of priority " << reaction_queue_pos_;

      // Make sure that any reaction is only executed once even if it
      // was triggered multiple times.
      std::sort(reactions.begin(), reactions.end());
      reactions.erase(std::unique(reactions.begin(), reactions.end()), reactions.end());

      if constexpr (log::debug_enabled || tracing_enabled) { // NOLINT
        for (auto* reaction : reactions) {
          log::Debug() << "(DefaultSchedulingPolicy) Reaction " << reaction->fqn() << " is ready for execution";
          tracepoint(reactor_cpp, trigger_reaction, reaction->container()->fqn(), reaction->name(), logical_time_);
        }
      }

      reactions_to_process_.store(static_cast<std::ptrdiff_t>(reactions.size()), std::memory_order_release);
      ready_queue_.fill_up(reactions);

      // break out of the loop and return
      return true;
    }
  }

  log::Debug() << "(DefaultSchedulingPolicy) Reached end of reaction queue";
  return false;
}

void DefaultSchedulingPolicy::terminate_all_workers() {
  log::Debug() << "(DefaultSchedulingPolicy) Send termination signal to all workers";
  auto num_workers = environment_.num_workers();
  std::vector<Reaction*> null_reactions{num_workers, nullptr};
  log::Debug() << null_reactions.size();
  ready_queue_.fill_up(null_reactions);
}

void DefaultSchedulingPolicy::trigger_reaction_from_next(Reaction* reaction) {
  reaction_queue_[reaction->index()].push_back(reaction);
}

void DefaultSchedulingPolicy::trigger_reaction_from_set_port(Reaction* reaction) {
  triggered_reactions_[Worker<DefaultSchedulingPolicy>::current_worker_id()].push_back(reaction);
}

auto DefaultSchedulingPolicy::ReadyQueue::pop() -> Reaction* {
  auto old_size = size_.fetch_sub(1, std::memory_order_acq_rel);

  // If there is no ready reaction available, wait until there is one.
  while (old_size <= 0) {
    log::Debug() << "(Worker " << Worker<DefaultSchedulingPolicy>::current_worker_id() << ") Wait for work";
    sem_.acquire();
    log::Debug() << "(Worker " << Worker<DefaultSchedulingPolicy>::current_worker_id() << ") Waking up";
    old_size = size_.fetch_sub(1, std::memory_order_acq_rel);
    // FIXME: Protect against underflow?
  }

  auto pos = old_size - 1;
  return queue_[pos];
}

void DefaultSchedulingPolicy::ReadyQueue::fill_up(std::vector<Reaction*>& ready_reactions) {
  // clear the internal queue and swap contents
  queue_.clear();
  queue_.swap(ready_reactions);

  // update the atomic size counter and release the semaphore to wake up
  // waiting worker threads
  auto new_size = static_cast<std::ptrdiff_t>(queue_.size());
  auto old_size = size_.exchange(new_size, std::memory_order_acq_rel);

  // calculate how many workers to wake up. -old_size indicates the number of
  // workers who started waiting since the last update.
  // We want to wake up at most all the waiting workers. If we would release
  // more, other workers that are out of work would not block when acquiring
  // the semaphore.
  // Also, we do not want to wake up more workers than there is work. new_size
  // indicates the number of ready reactions. Since there is always at least
  // one worker running running, new_size - running_workers indicates the
  // number of additional workers needed to process all reactions.
  waiting_workers_ += -old_size;
  auto running_workers = num_workers_ - waiting_workers_;
  auto workers_to_wakeup = std::min(waiting_workers_, new_size - running_workers);

  // wakeup other workers_
  if (workers_to_wakeup > 0) {
    waiting_workers_ -= workers_to_wakeup;
    log::Debug() << "Wakeup " << workers_to_wakeup << " workers";
    sem_.release(static_cast<int>(workers_to_wakeup));
  }
}

} // namespace reactor
