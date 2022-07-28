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

void GroupedSchedulingPolicy::init() {
  ReactionDependencyGraph graph{environment_.top_level_reactors()};
  ReactionDependencyGraph reduced_graph = graph.transitive_reduction();
  GroupedDependencyGraph grouped_graph{reduced_graph};
  grouped_graph.group_reactions_by_container(environment_.top_level_reactors());

  GroupedDependencyGraph reduced_grouped_graph = grouped_graph.transitive_reduction();
  reduced_grouped_graph.group_chains();

  auto g = reduced_grouped_graph.get_graph();

  std::map<GroupedDependencyGraph::GroupGraph::vertex_descriptor, std::shared_ptr<ReactionGroup>> vertex_to_group;

  // create a reaction group for each vertex and keep track of all groups without dependencies in initial_groups_
  for (auto* vertex : boost::make_iterator_range(vertices(g))) {
    auto group = std::make_shared<ReactionGroup>();
    vertex_to_group[vertex] = group;

    if (boost::in_degree(vertex, g) == 0) {
      initial_groups_.push_back(group);
    }
  }

  // initialize all reaction groups
  std::size_t id_counter{0};
  for (auto* vertex : boost::make_iterator_range(vertices(g))) {
    auto& group = vertex_to_group[vertex];
    group->id = id_counter++;

    // copy the list of reactions from the vertex
    group->reactions = boost::get(reduced_grouped_graph.get_group_property_map(), vertex);

    // Inform each reaction of its group and also set the index within the group
    std::size_t index_counter{0};
    for (auto* reaction : group->reactions) {
      reaction->set_scheduler_info(group);
      reaction->set_index(index_counter++);
    }

    // initialize the number of dependencies (in edges)
    std::size_t num_predecessors = boost::in_degree(vertex, g);
    group->waiting_for.store(num_predecessors, std::memory_order_release);
    group->num_predecessors = num_predecessors;

    // set all successors
    for (auto edge : boost::make_iterator_range(boost::out_edges(vertex, g))) {
      auto& successor = vertex_to_group[boost::target(edge, g)];
      group->successors.emplace_back(successor.get());
    }
  }

  log::Debug() << "Identified reaction groups: ";
  for (const auto& [_, group] : vertex_to_group) {
    log::Debug() << "* Group " << group->id << ':';
    log::Debug() << "   + reactions:";
    for (const auto* reaction : group->reactions) {
      log::Debug() << "      - " << reaction->fqn();
    }
    log::Debug() << "   + successors:";
    for (const auto* successor : group->successors) {
      log::Debug() << "      - Group " << successor->id;
    }
    log::Debug() << "   + num_predecessors: " << group->num_predecessors;
  }
}

auto GroupedSchedulingPolicy::create_worker() -> Worker<GroupedSchedulingPolicy> { return {*this, identity_counter++}; }

void GroupedSchedulingPolicy::worker_function(const Worker<GroupedSchedulingPolicy>& worker) {
  if (worker.id() == 0) {
    log::Debug() << "(Worker 0) do the initial scheduling";
    scheduler_.next();
  }
}

void GroupedSchedulingPolicy::trigger_reaction_from_next([[maybe_unused]] Reaction* reaction) { // NOLINT
  log::Warn() << "trigger_reaction_from_next is not yet implemented";
}
void GroupedSchedulingPolicy::trigger_reaction_from_set_port([[maybe_unused]] Reaction* reaction) { // NOLINT
  log::Warn() << "trigger_reaction_from_port is not yet implemented";
}

} // namespace reactor
