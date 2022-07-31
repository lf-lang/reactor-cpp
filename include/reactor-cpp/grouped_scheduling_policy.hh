/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_GROUPED_SCHEDULING_POLICY_HH
#define REACTOR_CPP_GROUPED_SCHEDULING_POLICY_HH

#include <cstddef>
#include <vector>

#include "reactor-cpp/fwd.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/semaphore.hh"

namespace reactor {

struct ReactionGroup {
  std::size_t id{0};
  std::vector<std::pair<bool, Reaction*>> reactions{};
  std::vector<ReactionGroup*> successors{};
  std::atomic<std::size_t> waiting_for{0};
  std::atomic<bool> triggered{false};
  std::size_t num_predecessors{0};

  std::shared_ptr<ReactionGroup> super_group{nullptr};
  std::vector<ReactionGroup*> sub_groups{};
  std::vector<ReactionGroup*> triggered_sub_groups{};
  std::atomic<std::size_t> triggered_sub_groups_write_pos{0};
};

class GroupedSchedulingPolicy {
private:
  std::size_t identity_counter{0};
  Scheduler<GroupedSchedulingPolicy>& scheduler_;
  Environment& environment_;

  std::vector<ReactionGroup*> initial_groups_;

  std::size_t num_groups_{0};
  std::atomic<std::size_t> groups_to_process_{0};
  std::atomic<bool> continue_execution_{true};

  static void process_group(const Worker<GroupedSchedulingPolicy>& worker, ReactionGroup* group);
  static void trigger_reaction(Reaction* reaction);

  class GroupQueue {
    // this vector only acts as a dynamically sized array
    std::vector<ReactionGroup*> queue_{};
    std::atomic<std::size_t> read_pos_{0};
    std::atomic<std::size_t> write_pos_{0};
    Semaphore semaphore_{0};

  public:
    void init(std::size_t max_size) { queue_.resize(max_size); }
    void reset();
    auto pop() -> ReactionGroup*;
    auto push(const std::vector<ReactionGroup*>& groups);
  };

  GroupQueue group_queue_;

  void schedule();
  auto finalize_group_and_notify_successors(ReactionGroup* group, std::vector<ReactionGroup*>& out_ready_groups)
      -> bool;
  void notify_groups(const std::vector<ReactionGroup*>& groups, std::vector<ReactionGroup*>& out_ready_groups);
  void notify_super_group(ReactionGroup* group, std::vector<ReactionGroup*>& out_ready_groups);
  void terminate_workers();

public:
  GroupedSchedulingPolicy(Scheduler<GroupedSchedulingPolicy>& scheduler, Environment& env);

  void init();
  auto create_worker() -> Worker<GroupedSchedulingPolicy>;
  void worker_function(const Worker<GroupedSchedulingPolicy>& worker);

  static inline void trigger_reaction_from_next(Reaction* reaction) { trigger_reaction(reaction); };
  static inline void trigger_reaction_from_set_port(Reaction* reaction) { trigger_reaction(reaction); };
};

} // namespace reactor

#endif // REACTOR_CPP_GROUPED_SCHEDULING_POLICY_HH
