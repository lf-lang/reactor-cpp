/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/grouped_scheduling_policy.hh"

#include "reactor-cpp/dependency_graph.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/scheduler.hh"

namespace reactor {

GroupedSchedulingPolicy::GroupedSchedulingPolicy(Scheduler<GroupedSchedulingPolicy>& scheduler, Environment& env)
    : scheduler_(scheduler)
    , environment_(env) {}

void GroupedSchedulingPolicy::init() {}

auto GroupedSchedulingPolicy::create_worker() -> Worker<GroupedSchedulingPolicy> { return {*this, identity_counter++}; }

void GroupedSchedulingPolicy::worker_function(const Worker<GroupedSchedulingPolicy>& worker) {
  reactor::log::Info() << "Hello from worker " << worker.id() << " [" << scheduler_.workers_.size() << "]\n";
}

void GroupedSchedulingPolicy::trigger_reaction_from_next([[maybe_unused]] Reaction* reaction) {}
void GroupedSchedulingPolicy::trigger_reaction_from_set_port([[maybe_unused]] Reaction* reaction) {}

} // namespace reactor
