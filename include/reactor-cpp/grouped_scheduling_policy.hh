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

class GroupedSchedulingPolicy {
private:
  Scheduler<GroupedSchedulingPolicy>& scheduler_;
  Environment& environment_;

public:
  GroupedSchedulingPolicy(Scheduler<GroupedSchedulingPolicy>& scheduler, Environment& env);

  void init();
  auto create_worker() -> Worker<GroupedSchedulingPolicy>;
  void worker_function(const Worker<GroupedSchedulingPolicy>& worker);

  void trigger_reaction_from_next(Reaction* reaction);
  void trigger_reaction_from_set_port(Reaction* reaction);
};

} // namespace reactor

#endif // REACTOR_CPP_GROUPED_SCHEDULING_POLICY_HH
