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
  std::atomic<bool> triggered{false};
  std::atomic<std::size_t> waiting_for{0};
  std::size_t num_predecessors{0};
};

class GroupedSchedulingPolicy {
private:
  std::size_t identity_counter{0};
  Scheduler<GroupedSchedulingPolicy>& scheduler_;
  Environment& environment_;

  std::vector<std::shared_ptr<ReactionGroup>> initial_groups_;

  static void trigger_reaction(Reaction* reaction);

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
