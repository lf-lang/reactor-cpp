/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_DEFAULT_SCHEDULING_POLICY_HH
#define REACTOR_CPP_DEFAULT_SCHEDULING_POLICY_HH

#include <cstddef>
#include <vector>

#include "reactor-cpp/fwd.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/semaphore.hh"

namespace reactor {

class DefaultSchedulingPolicy {
  Scheduler<DefaultSchedulingPolicy>& scheduler_;
  Environment& environment_;
  std::size_t identity_counter{0};

  class ReadyQueue {
  private:
    std::vector<Reaction*> queue_{};
    std::atomic<std::ptrdiff_t> size_{0};
    Semaphore sem_{0};
    std::ptrdiff_t waiting_workers_{0};
    const unsigned int num_workers_;

  public:
    explicit ReadyQueue(unsigned num_workers)
        : num_workers_(num_workers) {}

    /**
     * Retrieve a ready reaction from the queue.
     *
     * This method may be called concurrently. In case the queue is empty, the
     * method blocks and waits until a ready reaction becomes available.
     */
    auto pop() -> Reaction*;

    /**
     * Fill the queue up with ready reactions.
     *
     * This method assumes that the internal queue is empty. It moves all
     * reactions from the provided `ready_reactions` vector to the internal
     * queue, leaving `ready_reactions` empty.
     *
     * Note that this method is not thread-safe. The caller needs to ensure that
     * no other thread will try to read from the queue during this operation.
     */
    void fill_up(std::vector<Reaction*>& ready_reactions);
  };

  ReadyQueue ready_queue_;

  std::vector<std::vector<Reaction*>> reaction_queue_;
  unsigned int reaction_queue_pos_{std::numeric_limits<unsigned>::max()};

  std::atomic<std::ptrdiff_t> reactions_to_process_{0};
  std::vector<std::vector<Reaction*>> triggered_reactions_;

  bool continue_execution_{true};

  void schedule() noexcept;
  void terminate_all_workers();
  auto schedule_ready_reactions() -> bool;

public:
  DefaultSchedulingPolicy(Scheduler<DefaultSchedulingPolicy>& scheduler, Environment& env);

  void init();
  auto create_worker() -> Worker<DefaultSchedulingPolicy>;
  void worker_function(const Worker<DefaultSchedulingPolicy>& worker);

  void trigger_reaction_from_next(Reaction* reaction);
  void trigger_reaction_from_set_port(Reaction* reaction);
};

} // namespace reactor

#endif // REACTOR_CPP_DEFAULT_SCHEDULING_POLICY_HH
