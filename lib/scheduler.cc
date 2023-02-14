/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>

#include "reactor-cpp/scheduler.hh"

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/trace.hh"

namespace reactor {

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
thread_local const Worker* Worker::current_worker = nullptr;

Worker::Worker(Worker&& work) // NOLINT(performance-noexcept-move-constructor)
    : scheduler_{work.scheduler_}
    , identity_{work.identity_}
    , log_{std::move(work.log_)} {
  // Need to provide the move constructor in order to organize workers in a
  // std::vector. However, moving is not save if the thread is already running,
  // thus we throw an exception here if the worker is moved but the
  // internal thread is already running.

  if (work.thread_.joinable()) {
    throw std::runtime_error{"Running workers cannot be moved!"};
  }
}

void Worker::work() const {
  // initialize the current worker thread local variable
  current_worker = this;

  log_.debug() << "Starting";

  if (identity_ == 0) {
    log_.debug() << "do the initial scheduling";
    scheduler_.schedule();
  }

  while (true) {
    // wait for a ready reaction
    auto* reaction = scheduler_.ready_queue_.pop();

    // receiving a nullptr indicates that the worker should terminate
    if (reaction == nullptr) {
      break;
    }

    // execute the reaction
    execute_reaction(reaction);

    // was this the very last reaction?
    if (scheduler_.reactions_to_process_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      // Yes, then schedule. The atomic decrement above ensures that only one
      // thread enters this block.
      scheduler_.schedule();
    }
    // continue otherwise
  }

  log_.debug() << "terminates";
}

void Worker::execute_reaction(Reaction* reaction) const {
  log_.debug() << "execute reaction " << reaction->fqn();

  tracepoint(reactor_cpp, reaction_execution_starts, identity_, reaction->fqn(), scheduler_.logical_time());
  reaction->trigger();
  tracepoint(reactor_cpp, reaction_execution_finishes, identity_, reaction->fqn(), scheduler_.logical_time());
}

void Scheduler::schedule() noexcept {
  bool found_ready_reactions = schedule_ready_reactions();

  while (!found_ready_reactions) {
    log_.debug() << "call next()";
    next();
    reaction_queue_pos_ = 0;

    found_ready_reactions = schedule_ready_reactions();

    if (!continue_execution_ && !found_ready_reactions) {
      // let all workers know that they should terminate
      terminate_all_workers();
      break;
    }
  }
}

auto ReadyQueue::pop() -> Reaction* {
  auto old_size = size_.fetch_sub(1, std::memory_order_acq_rel);

  // If there is no ready reaction available, wait until there is one.
  while (old_size <= 0) {
    log_.debug() << "Worker " << Worker::current_worker_id() << " now waits for work";
    sem_.acquire();
    log_.debug() << "Worker " << Worker::current_worker_id() << " wakes up";
    old_size = size_.fetch_sub(1, std::memory_order_acq_rel);
    // FIXME: Protect against underflow?
  }

  auto pos = old_size - 1;
  return queue_[pos];
}

void ReadyQueue::fill_up(std::vector<Reaction*>& ready_reactions) {
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
  std::ptrdiff_t running_workers{num_workers_ - waiting_workers_};
  auto workers_to_wakeup = std::min(waiting_workers_, new_size - running_workers);

  // wakeup other workers_
  if (workers_to_wakeup > 0) {
    waiting_workers_ -= workers_to_wakeup;
    log_.debug() << "Wakeup " << workers_to_wakeup << " workers";
    sem_.release(static_cast<int>(workers_to_wakeup));
  }
}

void Scheduler::terminate_all_workers() {
  log_.debug() << "Send termination signal to all workers";
  auto num_workers = environment_->num_workers();
  std::vector<Reaction*> null_reactions{num_workers, nullptr};
  ready_queue_.fill_up(null_reactions);
}

auto Scheduler::schedule_ready_reactions() -> bool {
  // insert any triggered reactions_ into the reaction queue
  for (auto& vec_reaction : triggered_reactions_) {
    for (auto* reaction : vec_reaction) {
      reaction_queue_[reaction->index()].push_back(reaction);
    }
    vec_reaction.clear();
  }

  log_.debug() << "Scanning the reaction queue for ready reactions";

  // continue iterating over the reaction queue
  for (; reaction_queue_pos_ < reaction_queue_.size(); reaction_queue_pos_++) {
    auto& reactions = reaction_queue_[reaction_queue_pos_];

    // any ready reactions of current priority?
    if (!reactions.empty()) {
      log_.debug() << "Process reactions of priority " << reaction_queue_pos_;

      // Make sure that any reaction is only executed once even if it
      // was triggered multiple times.
      std::sort(reactions.begin(), reactions.end());
      reactions.erase(std::unique(reactions.begin(), reactions.end()), reactions.end());

      if constexpr (log::debug_enabled || tracing_enabled) { // NOLINT
        for (auto* reaction : reactions) {
          log_.debug() << "Reaction " << reaction->fqn() << " is ready for execution";
          tracepoint(reactor_cpp, trigger_reaction, reaction->container()->fqn(), reaction->name(), logical_time_);
        }
      }

      reactions_to_process_.store(static_cast<std::ptrdiff_t>(reactions.size()), std::memory_order_release);
      ready_queue_.fill_up(reactions);

      // break out of the loop and return
      return true;
    }
  }

  log_.debug() << "Reached end of reaction queue";
  return false;
}

void Scheduler::start() {
  log_.debug() << "Starting the scheduler...";

  auto num_workers = environment_->num_workers();
  // initialize the reaction queue, set ports vector, and triggered reactions
  // vector
  reaction_queue_.resize(environment_->max_reaction_index() + 1);
  set_ports_.resize(num_workers);
  triggered_reactions_.resize(num_workers);

  // Initialize and start the workers. By resizing the workers vector first,
  // we make sure that there is sufficient space for all the workers and non of
  // them needs to be moved. This is important because a running worker may not
  // be moved.
  workers_.reserve(num_workers);
  for (unsigned i = 0; i < num_workers; i++) {
    std::stringstream stream;
    stream << "Worker " << environment_->name() << " " << i;
    workers_.emplace_back(*this, i, stream.str());
    workers_.back().start_thread();
  }

  // join all worker threads
  for (auto& worker : workers_) {
    worker.join_thread();
  }
}

void Scheduler::next() { // NOLINT
  // clean up all actions triggered at the current tag (if there are any remaining)
  if (triggered_actions_ != nullptr) {
    // cleanup all triggered actions
    for (auto* action : *triggered_actions_) {
      action->cleanup();
    }

    triggered_actions_->clear();
    action_list_pool_.emplace_back(std::move(triggered_actions_));
  }

  // cleanup all set ports
  for (auto& vec_ports : set_ports_) {
    for (auto& port : vec_ports) {
      port->cleanup();
    }
    vec_ports.clear();
  }

  {
    std::unique_lock<std::mutex> lock{scheduling_mutex_};

    // shutdown if there are no more events in the queue
    if (event_queue_.empty() && !stop_) {
      if (environment_->run_forever()) {
        // wait for a new asynchronous event
        cv_schedule_.wait(lock, [this]() { return !event_queue_.empty() || stop_; });
      } else {
        log_.debug() << "No more events in queue_. -> Terminate!";
        environment_->sync_shutdown();
      }
    }

    while (triggered_actions_ == nullptr || triggered_actions_->empty()) {
      if (stop_) {
        continue_execution_ = false;
        log_.debug() << "Shutting down the scheduler";
        Tag t_next = Tag::from_logical_time(logical_time_).delay();
        if (!event_queue_.empty() && t_next == event_queue_.begin()->first) {
          log_.debug() << "Schedule the last round of reactions including all "
                          "termination reactions";
          triggered_actions_ = std::move(event_queue_.begin()->second);
          event_queue_.erase(event_queue_.begin());
          log_.debug() << "advance logical time to tag [" << t_next.time_point() << ", " << t_next.micro_step() << "]";
          logical_time_.advance_to(t_next);
        } else {
          return;
        }
      } else {
        // collect events of the next tag
        auto t_next = event_queue_.begin()->first;

        // synchronize with physical time if not in fast forward mode
        if (!environment_->fast_fwd_execution()) {
          // If physical time is smaller than the next logical time point,
          // then update the physical time. This step is small optimization to
          // avoid calling get_physical_time() in every iteration as this
          // would add a significant overhead.
          if (last_observed_physical_time_ < t_next.time_point()) {
            last_observed_physical_time_ = get_physical_time();
          }

          // If physical time is still smaller than the next logical time
          // point, then wait until the next tag or until a new event is
          // inserted asynchronously into the queue
          if (last_observed_physical_time_ < t_next.time_point()) {
            auto status = cv_schedule_.wait_until(lock, t_next.time_point());
            // Start over if an event was inserted into the event queue by a physical action
            if (status == std::cv_status::no_timeout || t_next != event_queue_.begin()->first) {
              continue;
            }
            // update physical time and continue otherwise
            last_observed_physical_time_ = t_next.time_point();
            reactor_assert(t_next == event_queue_.begin()->first);
          }
        }

        // retrieve all events with tag equal to current logical time from the
        // queue
        triggered_actions_ = std::move(event_queue_.extract(event_queue_.begin()).mapped());

        // advance logical time
        log_.debug() << "advance logical time to tag [" << t_next.time_point() << ", " << t_next.micro_step() << "]";
        logical_time_.advance_to(t_next);
      }
    }
  } // mutex schedule_

  // iterate over all events/actions, call setup and insert scheduled reactions
  log_.debug() << "events: " << triggered_actions_->size();
  for (auto* action : *triggered_actions_) {
    log_.debug() << "Action " << action->fqn();
    action->setup();
    for (auto* reaction : action->triggers()) {
      // There is no need to acquire the mutex. At this point the scheduler
      // should be the only thread accessing the reaction queue as none of the
      // workers_ are running
      log_.debug() << "insert reaction " << reaction->fqn() << " with index " << reaction->index();
      reaction_queue_[reaction->index()].push_back(reaction);
    }
  }
}

Scheduler::Scheduler(Environment* env)
    : using_workers_(env->num_workers() > 1)
    , environment_(env)
    , log_("Scheduler " + env->name())
    , ready_queue_(log_, env->num_workers()) {
  fill_action_list_pool();
}

Scheduler::~Scheduler() = default;

void Scheduler::fill_action_list_pool() {
  for (std::size_t i{0}; i < action_list_pool_increment_; i++) {
    action_list_pool_.emplace_back(std::make_unique<ActionList>());
  }
}

void Scheduler::schedule_sync(BaseAction* action, const Tag& tag) {
  log_.debug() << "Schedule action " << action->fqn() << (action->is_logical() ? " synchronously " : " asynchronously ")
               << " with tag [" << tag.time_point() << ", " << tag.micro_step() << "]";
  reactor_assert(logical_time_ < tag);
  tracepoint(reactor_cpp, schedule_action, action->container()->fqn(), action->name(), tag);

  if (using_workers_) {
    auto shared_lock = std::shared_lock<std::shared_mutex>(mutex_event_queue_);

    auto event_it = event_queue_.find(tag);
    if (event_it == event_queue_.end()) {
      shared_lock.unlock();
      {
        auto unique_lock = std::unique_lock<std::shared_mutex>(mutex_event_queue_);
        if (action_list_pool_.empty()) {
          fill_action_list_pool();
        }
        const auto& result = event_queue_.try_emplace(tag, std::move(action_list_pool_.back()));
        if (result.second) {
          action_list_pool_.pop_back();
        }
        result.first->second->push_back(action);
      }
    } else {
      event_it->second->push_back(action);
    }
  } else {
    if (action_list_pool_.empty()) {
      fill_action_list_pool();
    }
    const auto& result = event_queue_.try_emplace(tag, std::move(action_list_pool_.back()));
    if (result.second) {
      action_list_pool_.pop_back();
    }
    result.first->second->push_back(action);
  }
}

auto Scheduler::schedule_async(BaseAction* action, const Duration& delay) -> Tag {
  Tag tag;
  {
    std::lock_guard<std::mutex> lock_guard(scheduling_mutex_);
    tag = Tag::from_physical_time(get_physical_time() + delay);
    schedule_sync(action, tag);
  }
  cv_schedule_.notify_one();
  return tag;
}

void Scheduler::set_port(BasePort* port) {
  log_.debug() << "Set port " << port->fqn();

  // We do not check here if port is already in the list. This means clean()
  // could be called multiple times for a single port. However, calling
  // clean() multiple time is not harmful and more efficient then checking if
  set_ports_[Worker::current_worker_id()].push_back(port);

  // recursively search for triggered reactions
  set_port_helper(port);
}

void Scheduler::set_port_helper(BasePort* port) {
  // record the port for cleaning it up later
  set_ports_[Worker::current_worker_id()].push_back(port);

  // Call the 'set' callback on the port
  port->invoke_set_callback();

  // Record all triggered reactions
  for (auto* reaction : port->triggers()) {
    triggered_reactions_[Worker::current_worker_id()].push_back(reaction);
  }

  // Continue recursively
  for (auto* binding : port->outward_bindings()) {
    set_port_helper(binding);
  }
}

void Scheduler::stop() {
  stop_ = true;
  cv_schedule_.notify_one();
}

} // namespace reactor
