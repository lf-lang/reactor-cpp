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

void GroupedSchedulingPolicy::schedule() {
  group_queue_.reset();
  log::Debug() << "(Worker " << Worker<GroupedSchedulingPolicy>::current_worker_id() << ") call next";
  bool continue_execution = scheduler_.next();
  std::atomic_thread_fence(std::memory_order_release);
  if (!continue_execution) {
    continue_execution_.store(false, std::memory_order_relaxed);
  }
  groups_to_process_.store(num_groups_, std::memory_order_relaxed);
}

auto GroupedSchedulingPolicy::finalize_group_and_notify_successors(ReactionGroup* group,
                                                                   std::vector<ReactionGroup*>& out_ready_groups)
    -> bool {
  group->waiting_for.store(group->num_predecessors, std::memory_order_release);
  notify_groups(group->successors, out_ready_groups);

  // return true if the group was the last to be processed.
  return 1 == groups_to_process_.fetch_sub(1, std::memory_order_acq_rel);
}

void GroupedSchedulingPolicy::notify_groups(const std::vector<ReactionGroup*>& groups,
                                            std::vector<ReactionGroup*>& out_ready_groups) {
  for (auto* group : groups) {
    // decrement the waiting for counter
    auto old = group->waiting_for.fetch_sub(1, std::memory_order_relaxed);
    // If the old value was 1 (or 0), then all dependencies are fulfilled and the group is ready for execution
    if (old <= 1) {
      // If the group was triggered, then add it to the ready queue. Otherwise, we skip the group and check its
      // successors.
      if (group->triggered.exchange(false, std::memory_order_relaxed)) {
        out_ready_groups.emplace_back(group);
      } else {
        finalize_group_and_notify_successors(group, out_ready_groups);
      }
    }
  }
  std::atomic_thread_fence(std::memory_order_acquire);
}

void GroupedSchedulingPolicy::terminate_workers() {
  log::Debug() << "(Worker " << Worker<GroupedSchedulingPolicy>::current_worker_id()
               << ") signal all workers to terminate";
  std::vector<ReactionGroup*> null_groups_(environment_.num_workers() + 1, nullptr);
  group_queue_.push(null_groups_);
  groups_to_process_.store(environment_.num_workers(), std::memory_order_release);
}

void GroupedSchedulingPolicy::worker_function(const Worker<GroupedSchedulingPolicy>& worker) {
  // This is used as a list for storing new ready groups found while processing a group.
  std::vector<ReactionGroup*> ready_groups;
  ready_groups.reserve(num_groups_);

  // We use this variable to pass a group to process to ourselves (avoiding the queue)
  ReactionGroup* next_group{nullptr};

  // Worker 0 does the initial scheduling
  if (worker.id() == 0) {
    log::Debug() << "(Worker 0) do the initial scheduling";
    schedule();
    notify_groups(initial_groups_, ready_groups);
    group_queue_.push(ready_groups);
    ready_groups.clear();
  }

  while (true) {
    // Get a group to process. If we set next_group in the last iteration, we
    // process this group. Otherwise we pop a group from the queue.
    auto* group = next_group != nullptr ? next_group : group_queue_.pop();
    next_group = nullptr;

    // receiving a nullptr indicates that the worker should terminate
    if (group == nullptr) {
      break;
    }

    // process the group
    process_group(worker, group);
    bool need_to_schedule = finalize_group_and_notify_successors(group, ready_groups);

    if (need_to_schedule) {
      // We use a do-while loop here as we could have scheduled events that do not trigger any reactions.
      // In this case, ready_groups will be empty and we can simply call schedule again.
      do {
        if (continue_execution_.load(std::memory_order_acquire)) {
          schedule();
          notify_groups(initial_groups_, ready_groups);
        } else {
          terminate_workers();
          break;
        }
      } while (ready_groups.empty());
    }

    log::Debug() << "(Worker " << worker.id() << ") found " << ready_groups.size()
                 << " new groups that are ready for execution";
    if (!ready_groups.empty()) {
      // take one group for ourselves
      next_group = ready_groups.back();
      ready_groups.pop_back();

      // if there are more, we put them on the queue
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
  group->triggered.store(true, std::memory_order_release);
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
