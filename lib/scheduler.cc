/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

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
    , identity_{work.identity_} {
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

  log::Debug() << "(Worker " << this->identity_ << ") Starting";

  if (identity_ == 0) {
    log::Debug() << "(Worker 0) do the initial scheduling";
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

  log::Debug() << "(Worker " << identity_ << ") terminates";
}

void Worker::execute_reaction(Reaction* reaction) const {
  log::Debug() << "(Worker " << identity_ << ") "
               << "execute reaction " << reaction->fqn();

  tracepoint(reactor_cpp, reaction_execution_starts, id, reaction->fqn(), scheduler.logical_time());
  reaction->trigger();
  tracepoint(reactor_cpp, reaction_execution_finishes, id, reaction->fqn(), scheduler.logical_time());
}

void Scheduler::schedule() noexcept {
  bool found_ready_reactions = schedule_ready_reactions();

  while (!found_ready_reactions) {
    log::Debug() << "(Scheduler) call next()";
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
    log::Debug() << "(Worker " << Worker::current_worker_id() << ") Wait for work";
    sem_.acquire();
    log::Debug() << "(Worker " << Worker::current_worker_id() << ") Waking up";
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
  auto running_workers = num_workers_ - waiting_workers_;
  auto workers_to_wakeup = std::min(waiting_workers_, new_size - running_workers);

  // wakeup other workers_
  if (workers_to_wakeup > 0) {
    waiting_workers_ -= workers_to_wakeup;
    log::Debug() << "Wakeup " << workers_to_wakeup << " workers";
    sem_.release(static_cast<int>(workers_to_wakeup));
  }
}

void Scheduler::terminate_all_workers() {
  log::Debug() << "(Scheduler) Send termination signal to all workers";
  auto num_workers = environment_->num_workers();
  std::vector<Reaction*> null_reactions{num_workers, nullptr};
  log::Debug() << null_reactions.size();
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

  log::Debug() << "(Scheduler) Scanning the reaction queue for ready reactions";

  // continue iterating over the reaction queue
  for (; reaction_queue_pos_ < reaction_queue_.size(); reaction_queue_pos_++) {
    auto& reactions = reaction_queue_[reaction_queue_pos_];

    // any ready reactions of current priority?
    if (!reactions.empty()) {
      log::Debug() << "(Scheduler) Process reactions of priority " << reaction_queue_pos_;

      // Make sure that any reaction is only executed once even if it
      // was triggered multiple times.
      std::sort(reactions.begin(), reactions.end());
      reactions.erase(std::unique(reactions.begin(), reactions.end()), reactions.end());

      if constexpr (log::debug_enabled || tracing_enabled) { // NOLINT
        for (auto* reaction : reactions) {
          log::Debug() << "(Scheduler) Reaction " << reaction->fqn() << " is ready for execution";
          tracepoint(reactor_cpp, trigger_reaction, reaction->container()->fqn(), reaction->name(), logical_time_);
        }
      }

      reactions_to_process_.store(static_cast<std::ptrdiff_t>(reactions.size()), std::memory_order_release);
      ready_queue_.fill_up(reactions);

      // break out of the loop and return
      return true;
    }
  }

  log::Debug() << "(Scheduler) Reached end of reaction queue";
  return false;
}

void Scheduler::start() {
  log::Debug() << "Starting the scheduler...";

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
    workers_.emplace_back(*this, i);
    workers_.back().start_thread();
  }

  // join all worker threads
  for (auto& worker : workers_) {
    worker.join_thread();
  }
}

void Scheduler::next() { // NOLINT
  static EventMap events{};

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
        continue_execution_ = false;
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
          return;
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
      log::Debug() << "insert reaction " << reaction->fqn() << " with index " << reaction->index();
      reaction_queue_[reaction->index()].push_back(reaction);
    }
  }
}

Scheduler::Scheduler(Environment* env)
    : using_workers_(env->num_workers() > 1)
    , environment_(env)
    , ready_queue_(env->num_workers()) {}

Scheduler::~Scheduler() = default;

void Scheduler::schedule_sync(const Tag& tag, BaseAction* action, std::function<void(void)> pre_handler) {
  reactor_assert(logical_time_ < tag);
  // TODO verify that the action is indeed allowed to be scheduled by the
  // current reaction
  log::Debug() << "Schedule action " << action->fqn() << (action->is_logical() ? " synchronously " : " asynchronously ")
               << " with tag [" << tag.time_point() << ", " << tag.micro_step() << "]";
  {
    auto unique_lock =
        using_workers_ ? std::unique_lock<std::mutex>(lock_event_queue_) : std::unique_lock<std::mutex>();

    tracepoint(reactor_cpp, schedule_action, action->container()->fqn(), action->name(), tag); // NOLINT

    // create a new event map or retrieve the existing one
    auto emplace_result = event_queue_.try_emplace(tag, EventMap());
    auto& event_map = emplace_result.first->second;

    // insert the new event
    event_map[action] = std::move(pre_handler);
  }
}

void Scheduler::schedule_async(const Tag& tag, BaseAction* action, std::function<void(void)> pre_handler) {
  std::lock_guard<std::mutex> lock_guard(scheduling_mutex_);
  schedule_sync(tag, action, std::move(pre_handler));
  cv_schedule_.notify_one();
}

void Scheduler::set_port(BasePort* port) {
  log::Debug() << "Set port " << port->fqn();

  // We do not check here if port is already in the list. This means clean()
  // could be called multiple times for a single port. However, calling
  // clean() multiple time is not harmful and more efficient then checking if
  set_ports_[Worker::current_worker_id()].push_back(port);

  // recursively search for triggered reactions
  set_port_helper(port);
}

void Scheduler::set_port_helper(BasePort* port) {
  if (!(port->triggers().empty() && port->dependencies().empty())) {
    if (port->activate()) {
        set_ports_[Worker::current_worker_id()].push_back(port);
    }
  }

  for (auto* reaction : port->triggers()) {
    triggered_reactions_[Worker::current_worker_id()].push_back(reaction);
  }
  for (auto* binding : port->outward_bindings()) {
    set_port_helper(binding);
  }
}

void Scheduler::stop() {
  stop_ = true;
  cv_schedule_.notify_one();
}

} // namespace reactor
