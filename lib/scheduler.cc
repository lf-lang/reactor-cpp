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

thread_local const Worker* Worker::current_worker{nullptr};

Worker::Worker(Worker&& w) : scheduler{w.scheduler}, id{w.id}, thread{} {
  // Need to provide the move constructor in order to organize workers in a
  // std::vector. However, moving is not save if the thread is already running,
  // thus we throw an exception here if the the worker is moved but the
  // internal thread is already running.

  if (w.thread.joinable()) {
    throw std::runtime_error{"Running workers cannot be moved!"};
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
    auto reaction = scheduler.ready_queue.pop();

    // receiving a nullptr indicates that the worker should terminate
    if (reaction == nullptr) {
      break;
    }

    // execute the reaction
    execute_reaction(reaction);

    // was this the very last reaction?
    if (scheduler.reactions_to_process.fetch_sub(
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
    reaction_queue_pos = 0;

    found_ready_reactions = schedule_ready_reactions();

    if (!continue_execution && !found_ready_reactions) {
      // let all workers know that they should terminate
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

  // calculate how many workers to wake up. -old_size indicates the number of
  // workers who started waiting since the last update.
  // We want to wake up at most all the waiting workers. If we would release
  // more, other workers that are out of work would not block when acquiring
  // the semaphore.
  // Also, we do not want to wake up more workers than there is work. new_size
  // indicates the number of ready reactions. Since there is always at least
  // one worker running running, new_size - running_workers indicates the
  // number of additional workers needed to process all reactions.
  waiting_workers += -old_size;
  auto running_workers = num_workers - waiting_workers;
  auto workers_to_wakeup =
      std::min(waiting_workers, new_size - running_workers);

  // wakeup other workers
  if (workers_to_wakeup > 0) {
    waiting_workers -= workers_to_wakeup;
    log::Debug() << "Wakeup " << workers_to_wakeup << " workers";
    sem.release((unsigned)workers_to_wakeup);
  }
}

void Scheduler::terminate_all_workers() {
  log::Debug() << "(Scheduler) Send termination signal to all workers";
  auto num_workers = _environment->num_workers();
  std::vector<Reaction*> null_reactions{num_workers, nullptr};
  log::Debug() << null_reactions.size();
  ready_queue.fill_up(null_reactions);
}

bool Scheduler::schedule_ready_reactions() {
  // insert any triggered reactions into the reaction queue
  for (auto& v : triggered_reactions) {
    for (auto n : v) {
      reaction_queue[n->index()].push_back(n);
    }
    v.clear();
  }

  log::Debug() << "(Scheduler) "
               << "Scanning the reaction queue for ready reactions";

  // continue iterating over the reaction queue
  for (; reaction_queue_pos < reaction_queue.size(); reaction_queue_pos++) {
    auto& reactions = reaction_queue[reaction_queue_pos];

    // any ready reactions of current priority?
    if (!reactions.empty()) {
      log::Debug() << "(Scheduler) Process reactions of priority "
                   << reaction_queue_pos;

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
                     r->name(), _logical_time);
        }
      }

      reactions_to_process.store(reactions.size(), std::memory_order_release);
      ready_queue.fill_up(reactions);

      // break out of the loop and return
      return true;
    }
  }

  log::Debug() << "(Scheduler) Reached end of reaction queue";

  return false;
}

void Scheduler::start() {
  log::Debug() << "Starting the scheduler...";

  auto num_workers = _environment->num_workers();
  // initialize the reaction queue, set ports vector, and triggered reactions
  // vector
  reaction_queue.resize(_environment->max_reaction_index() + 1);
  set_ports.resize(num_workers);
  triggered_reactions.resize(num_workers);

  // Initialize and start the workers. By resizing the workers vector first, we
  // make sure that there is sufficient space for all the workers and non of
  // them needs to be moved. This is important because a running worker may not
  // be moved.
  workers.reserve(num_workers);
  for (unsigned i = 0; i < num_workers; i++) {
    workers.emplace_back(*this, i);
    workers.back().start_thread();
  }

  // join all worker threads
  for (auto& w : workers) {
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
    for (auto& v : set_ports) {
      for (auto& p : v) {
        p->cleanup();
      }
      v.clear();
    }
    events.clear();
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

    while (events.empty()) {
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
      // workers are running
      log::Debug() << "insert reaction " << n->fqn() << " with index "
                   << n->index();
      reaction_queue[n->index()].push_back(n);
    }
  }
}

Scheduler::Scheduler(Environment* env)
    : using_workers(env->num_workers() > 1)
    , _environment(env)
    , ready_queue(env->num_workers()) {}

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
    auto emplace_result = event_queue.try_emplace(tag, EventMap());
    auto& event_map = emplace_result.first->second;

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

  // We do not check here if p is already in the list. This means clean()
  // could be called multiple times for a single port. However, calling
  // clean() multiple time is not harmful and more efficient then checking if
  set_ports[Worker::current_worker_id()].push_back(p);

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
      triggered_reactions[Worker::current_worker_id()].push_back(n);
    }
  }
}

void Scheduler::stop() {
  _stop = true;
  cv_schedule.notify_one();
}

}  // namespace reactor
