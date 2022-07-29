/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/grouped_scheduling_policy.hh"

#include "reactor-cpp/assert.hh"
#include "reactor-cpp/dependency_graph.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/scheduler.hh"

#include <algorithm>
#include <atomic>
#include <iterator>
#include <vector>

namespace reactor {

auto GroupedSchedulingPolicy::GroupQueue::pop() -> ReactionGroup* {
  log::Debug() << "(Worker " << Worker<GroupedSchedulingPolicy>::current_worker_id() << ") Wait for work";
  semaphore_.acquire();
  log::Debug() << "(Worker " << Worker<GroupedSchedulingPolicy>::current_worker_id() << ") Waking up";

  auto pos = read_pos_.fetch_add(1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_acquire);
  reactor_assert(pos < queue_.size());
  return queue_[pos];
}

auto GroupedSchedulingPolicy::GroupQueue::push(const std::vector<ReactionGroup*>& groups) {
  auto pos = write_pos_.fetch_add(groups.size(), std::memory_order_relaxed);
  for (auto* group : groups) {
    reactor_assert(pos < queue_.size());
    queue_[pos++] = group;
  }
  std::atomic_thread_fence(std::memory_order_acquire);
  semaphore_.release(groups.size());
}

void GroupedSchedulingPolicy::GroupQueue::reset() {
  std::atomic_thread_fence(std::memory_order_release);
  read_pos_.store(0, std::memory_order_relaxed);
  write_pos_.store(0, std::memory_order_relaxed);
  std::fill(queue_.begin(), queue_.end(), nullptr);
}

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
      initial_groups_.push_back(group.get());
    }
  }

  // initialize all reaction groups
  std::size_t id_counter{0};
  for (auto* vertex : boost::make_iterator_range(vertices(g))) {
    auto& group = vertex_to_group[vertex];
    group->id = id_counter++;

    // copy the list of reactions from the vertex
    for (auto* reaction : boost::get(reduced_grouped_graph.get_group_property_map(), vertex)) {
      group->reactions.emplace_back(std::make_pair(false, reaction));
    }

    // Inform each reaction of its group and also set the index within the group
    std::size_t index_counter{0};
    for (auto [_, reaction] : group->reactions) {
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

  num_groups_ = vertex_to_group.size();
  group_queue_.init(std::max(num_groups_, environment_.num_workers() + 1));

  log::Debug() << "Identified reaction groups: ";
  for (const auto& [_, group] : vertex_to_group) {
    log::Debug() << "* Group " << group->id << ':';
    log::Debug() << "   + reactions:";
    for (auto [_, reaction] : group->reactions) {
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

void GroupedSchedulingPolicy::worker_function(const Worker<GroupedSchedulingPolicy>& worker) { // NOLINT
  if (worker.id() == 0) {
    log::Debug() << "(Worker 0) do the initial scheduling";
    scheduler_.next();
    groups_to_process_.store(num_groups_, std::memory_order_release);
    group_queue_.push(initial_groups_);
  }

  // This is used as a list for storing new ready groups while processing a group.
  std::vector<ReactionGroup*> ready_groups;
  ready_groups.reserve(num_groups_);

  // We use this variable to pass a group to ourselves (avoiding the queue)
  ReactionGroup* next_group{nullptr};

  while (true) {
    auto* group = next_group != nullptr ? next_group : group_queue_.pop();
    // receiving a nullptr indicates that the worker should terminate
    if (group == nullptr) {
      break;
    }

    // first, process the group
    process_group(worker, group);

    // Check if this was the last group.
    // If so, we call next() again, otherwise we update all successors and check for new ready groups
    bool call_next{1 == groups_to_process_.fetch_sub(1, std::memory_order_acq_rel)};
    if (call_next) {
      group_queue_.reset();
      if (continue_execution_.load(std::memory_order_acquire)) {
        log::Debug() << "(Worker " << worker.id() << ") call next";
        if (!scheduler_.next()) {
          continue_execution_.store(false, std::memory_order_release);
        }
        std::copy(initial_groups_.begin(), initial_groups_.end(), std::back_inserter(ready_groups));
        groups_to_process_.store(num_groups_, std::memory_order_release);
      } else {
        std::vector<ReactionGroup*> null_groups_(environment_.num_workers() + 1, nullptr);
        group_queue_.push(null_groups_);
        groups_to_process_.store(environment_.num_workers(), std::memory_order_release);
      }
    } else {
      // Check if any of the successors has become ready
      for (auto* successor : group->successors) {
        auto old = successor->waiting_for.fetch_sub(1, std::memory_order_relaxed);
        if (old == 1) {
          ready_groups.emplace_back(successor);
        }
      }
    }

    // reset the waiting_for counter of the current group
    std::atomic_thread_fence(std::memory_order_acq_rel);
    group->waiting_for.store(group->num_predecessors, std::memory_order_relaxed);

    if (ready_groups.empty()) {
      next_group = nullptr;
    } else {
      log::Debug() << "(Worker " << worker.id() << ") found " << ready_groups.size()
                   << " new groups that are ready for execution";
      // keep the first ready group for ourselves to process next
      next_group = ready_groups.back();
      ready_groups.pop_back();

      // all other groups we give to the ready queue
      if (!ready_groups.empty()) {
        group_queue_.push(ready_groups);
      }
      ready_groups.clear();
    }
  }
}

void GroupedSchedulingPolicy::trigger_reaction(Reaction* reaction) {
  auto group = reaction->get_scheduler_info<ReactionGroup>();
  log::Debug() << "(GroupedSchedulingPolicy) trigger reaction " << reaction->fqn() << " in Group " << group->id;
  auto& triggered_reaction_pair = group->reactions[reaction->index()];
  triggered_reaction_pair.first = true;
}

void GroupedSchedulingPolicy::process_group(const Worker<GroupedSchedulingPolicy>& worker, ReactionGroup* group) {
  log::Debug() << "(Worker " << worker.id() << ") process Group " << group->id;
  for (auto& triggered_reaction_pair : group->reactions) {
    if (triggered_reaction_pair.first) {
      triggered_reaction_pair.first = false;
      worker.execute_reaction(triggered_reaction_pair.second);
    }
  }
}
} // namespace reactor
