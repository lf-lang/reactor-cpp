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

thread_local const Worker* Worker::current_worker{nullptr};

Worker::Worker(Worker&& w): scheduler{w.scheduler}, id{w.id}, thread{} {
  // Need to provide the move constructor in order to organize workers_ in a
  // std::vector. However, moving is not save if the thread is already running,
  // thus we throw an exception here if the the worker is moved but the
  // internal thread is already running.

  if (w.thread.joinable()) {
    throw std::runtime_error{"Running workers_ cannot be moved!"};
  }
}

void Worker::work() const {
  // initialize the current worker thread local variable
  current_worker = this;

  log::Debug() << "(Worker " << this->id << ") Starting";

  if (id == 0) {
    log::Debug() << "(Worker 0) do the initial scheduling";
    scheduler.schedule();
  }

  while (true) {
    // wait for a ready reaction
    auto reaction = scheduler.ready_queue_.pop();

    // receiving a nullptr indicates that the worker should terminate
    if (reaction == nullptr) {
      break;
    }

    // execute the reaction
    execute_reaction(reaction);

    // was this the very last reaction?
    if (scheduler.reactions_to_process_.fetch_sub(
            1, std::memory_order_acq_rel) == 1) {
      // Yes, then schedule. The atomic decrement above ensures that only one
      // thread enters this block.
      scheduler.schedule();
    }
    // continue otherwise
  }

  log::Debug() << "(Worker " << id << ") terminates";
}

void Worker::execute_reaction(Reaction* reaction) const {
  log::Debug() << "(Worker " << id << ") "
               << "execute reaction " << reaction->fqn();
  tracepoint(reactor_cpp, reaction_execution_starts, id, reaction->fqn());
  reaction->trigger();
  tracepoint(reactor_cpp, reaction_execution_finishes, id, reaction->fqn());
}

void Scheduler::schedule() {
  bool found_ready_reactions = schedule_ready_reactions();

  while (!found_ready_reactions) {
    log::Debug() << "(Scheduler) call next()";
    next();
      reaction_queue_pos_ = 0;

    found_ready_reactions = schedule_ready_reactions();

    if (!continue_execution_ && !found_ready_reactions) {
      // let all workers_ know that they should terminate
      terminate_all_workers();
      break;
    }
  }
}

Reaction* ReadyQueue::pop() {
  auto old_size = size.fetch_sub(1, std::memory_order_acq_rel);

  // If there is no ready reaction available, wait until there is one.
  while (old_size <= 0) {
    log::Debug() << "(Worker " << Worker::current_worker_id()
                 << ") Wait for work";
    sem.acquire();
    log::Debug() << "(Worker " << Worker::current_worker_id() << ") Waking up";
    old_size = size.fetch_sub(1, std::memory_order_acq_rel);
    // FIXME: Protect against underflow?
  }

  auto pos = old_size - 1;
  return queue[pos];
}

void ReadyQueue::fill_up(std::vector<Reaction*>& ready_reactions) {
  // clear the internal queue and swap contents
  queue.clear();
  queue.swap(ready_reactions);

  // update the atomic size counter and release the semaphore to wake up
  // waiting worker threads
  std::ptrdiff_t new_size = queue.size();
  auto old_size = size.exchange(new_size, std::memory_order_acq_rel);

  // calculate how many workers_ to wake up. -old_size indicates the number of
  // workers_ who started waiting since the last update.
  // We want to wake up at most all the waiting workers_. If we would release
  // more, other workers_ that are out of work would not block when acquiring
  // the semaphore.
  // Also, we do not want to wake up more workers_ than there is work. new_size
  // indicates the number of ready reactions_. Since there is always at least
  // one worker running running, new_size - running_workers indicates the
  // number of additional workers_ needed to process all reactions_.
  waiting_workers += -old_size;
  auto running_workers = num_workers - waiting_workers;
  auto workers_to_wakeup =
      std::min(waiting_workers, new_size - running_workers);

  // wakeup other workers_
  if (workers_to_wakeup > 0) {
    waiting_workers -= workers_to_wakeup;
    log::Debug() << "Wakeup " << workers_to_wakeup << " workers_";
    sem.release((unsigned)workers_to_wakeup);
  }
}

void Scheduler::terminate_all_workers() {
  log::Debug() << "(Scheduler) Send termination signal to all workers_";
  auto num_workers = environment_->num_workers();
  std::vector<Reaction*> null_reactions{num_workers, nullptr};
  log::Debug() << null_reactions.size();
  ready_queue_.fill_up(null_reactions);
}

bool Scheduler::schedule_ready_reactions() {
  // insert any triggered reactions_ into the reaction queue
  for (auto& v : triggered_reactions_) {
    for (auto n : v) {
      reaction_queue_[n->index()].push_back(n);
    }
    v.clear();
  }

  log::Debug() << "(Scheduler) "
               << "Scanning the reaction queue for ready reactions_";

  // continue iterating over the reaction queue
  for (; reaction_queue_pos_ < reaction_queue_.size(); reaction_queue_pos_++) {
    auto& reactions = reaction_queue_[reaction_queue_pos_];

    // any ready reactions_ of current priority?
    if (!reactions.empty()) {
      log::Debug() << "(Scheduler) Process reactions_ of priority "
                   << reaction_queue_pos_;

      // Make sure that any reaction is only executed once even if it
      // was triggered multiple times.
      std::sort(reactions.begin(), reactions.end());
      reactions.erase(std::unique(reactions.begin(), reactions.end()),
                      reactions.end());

      if constexpr (log::debug_enabled || tracing_enabled) {
        for (auto r : reactions) {
          log::Debug() << "(Scheduler) Reaction " << r->fqn()
                       << " is ready for execution";
          tracepoint(reactor_cpp, trigger_reaction, r->container()->fqn(),
                     r->name(), logical_time_);
        }
      }

      reactions_to_process_.store(reactions.size(), std::memory_order_release);
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
  // initialize the reaction queue, set ports vector, and triggered reactions_
  // vector
  reaction_queue_.resize(environment_->max_reaction_index() + 1);
  set_ports_.resize(num_workers);
  triggered_reactions_.resize(num_workers);

  // Initialize and start the workers_. By resizing the workers_ vector first, we
  // make sure that there is sufficient space for all the workers_ and non of
  // them needs to be moved. This is important because a running worker may not
  // be moved.
  workers_.reserve(num_workers);
  for (unsigned i = 0; i < num_workers; i++) {
    workers_.emplace_back(*this, i);
    workers_.back().start_thread();
  }

  // join all worker threads
  for (auto& w : workers_) {
    w.join_thread();
  }
}

void Scheduler::next() {
  static EventMap events{};

  // clean up before scheduling any new events
  if (!events.empty()) {
    // cleanup all triggered actions
    for (auto& kv : events) {
      kv.first->cleanup();
    }
    // cleanup all set ports
    for (auto& v : set_ports_) {
      for (auto& p : v) {
        p->cleanup();
      }
      v.clear();
    }
    events.clear();
  }

  {
    std::unique_lock<std::mutex> lock{schedule_};

    // shutdown if there are no more events in the queue
    if (event_queue_.empty() && !stop_) {
      if (environment_->run_forever()) {
        // wait for a new asynchronous event
        cv_schedule_.wait(lock,
                          [this]() { return !event_queue_.empty() || stop_; });
      } else {
        log::Debug() << "No more events in queue. -> Terminate!";
        environment_->sync_shutdown();
      }
    }

    while (events.empty()) {
      if (stop_) {
          continue_execution_ = false;
        log::Debug() << "Shutting down the scheduler";
        Tag t_next = Tag::from_logical_time(logical_time_).delay();
        if (t_next == event_queue_.begin()->first) {
          log::Debug() << "Schedule the last round of reactions_ including all "
                          "termination reactions_";
          events = std::move(event_queue_.begin()->second);
          event_queue_.erase(event_queue_.begin());
          log::Debug() << "advance logical time to tag [" << t_next.time_point()
                       << ", " << t_next.micro_step() << "]";
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
          if (physical_time < t_next.time_point())
            physical_time = get_physical_time();

          // If physical time is still smaller than the next logical time
          // point, then wait until the next tag or until a new event is
          // inserted asynchronously into the queue
          if (physical_time < t_next.time_point()) {
            auto status = cv_schedule_.wait_until(lock, t_next.time_point());
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
        events = std::move(event_queue_.begin()->second);
        event_queue_.erase(event_queue_.begin());

        // advance logical time
        log::Debug() << "advance logical time to tag [" << t_next.time_point()
                     << ", " << t_next.micro_step() << "]";
        logical_time_.advance_to(t_next);
      }
    }
  }  // mutex schedule_

  // execute all setup functions; this sets the values of the corresponding
  // actions
  for (auto& kv : events) {
    auto& setup = kv.second;
    if (setup != nullptr) {
      setup();
    }
  }

  log::Debug() << "events: " << events.size();
  for (auto& kv : events) {
    log::Debug() << "Action " << kv.first->fqn();
    for (auto n : kv.first->triggers()) {
      // There is no need to acquire the mutex. At this point the scheduler
      // should be the only thread accessing the reaction queue as none of the
      // workers_ are running
      log::Debug() << "insert reaction " << n->fqn() << " with index "
                   << n->index();
      reaction_queue_[n->index()].push_back(n);
    }
  }
}

Scheduler::Scheduler(Environment* env): using_workers_(env->num_workers() > 1)
    , environment_(env)
    , ready_queue_(env->num_workers()) {}

Scheduler::~Scheduler() = default;

void Scheduler::schedule_sync(const Tag& tag,
                              BaseAction* action,
                              std::function<void(void)> setup) {
    reactor_assert(logical_time_ < tag);
    // TODO verify that the action is indeed allowed to be scheduled by the
    // current reaction
    log::Debug() << "Schedule action " << action->fqn()
               << (action->is_logical() ? " synchronously "
                                        : " asynchronously ")
               << " with tag [" << tag.time_point() << ", " << tag.micro_step()
               << "]";
    {
        auto lg = using_workers_ ? std::unique_lock<std::mutex>(lock_event_queue_)
                                 : std::unique_lock<std::mutex>();

        tracepoint(reactor_cpp, schedule_action, action->container()->fqn(),
               action->name(), tag);

        // create a new event map or retrieve the existing one
        auto emplace_result = event_queue_.try_emplace(tag, EventMap());
        auto& event_map = emplace_result.first->second;

        // insert the new event
        event_map[action] = std::move(setup);
    }
}

void Scheduler::schedule_async(const Tag& tag,
                               BaseAction* action,
                               std::function<void(void)> setup) {
    std::lock_guard<std::mutex> lg(schedule_);
    schedule_sync(tag, action, std::move(setup));
    cv_schedule_.notify_one();
}

void Scheduler::set_port(BasePort* p) {
    log::Debug() << "Set port " << p->fqn();

    // We do not check here if p is already in the list. This means clean()
    // could be called multiple times for a single port. However, calling
    // clean() multiple time is not harmful and more efficient then checking if
    set_ports_[Worker::current_worker_id()].push_back(p);

    // recursively search for triggered reactions_
    set_port_helper(p);
}

void Scheduler::set_port_helper(BasePort* p) {
    reactor_assert(!(p->has_outward_bindings() && !p->triggers().empty()));
    if (p->has_outward_bindings()) {
        for (auto binding : p->outward_bindings()) {
            set_port_helper(binding);
        }
    } else {
        for (auto n : p->triggers()) {
            triggered_reactions_[Worker::current_worker_id()].push_back(n);
        }
    }
}

void Scheduler::stop() {
    stop_ = true;
    cv_schedule_.notify_one();
}

}  // namespace reactor
