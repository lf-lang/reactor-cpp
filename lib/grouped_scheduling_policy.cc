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
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/scheduler.hh"

#include <algorithm>
#include <atomic>
#include <cstddef>
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

void GroupedSchedulingPolicy::init() { // NOLINT
  ReactionDependencyGraph graph{environment_.top_level_reactors()};
  ReactionDependencyGraph reduced_graph = graph.transitive_reduction();
  GroupedDependencyGraph grouped_graph{reduced_graph};
  grouped_graph.group_reactions_by_container(environment_.top_level_reactors());

  GroupedDependencyGraph reduced_grouped_graph = grouped_graph.transitive_reduction();
  reduced_grouped_graph.group_chains();

  auto g = reduced_grouped_graph.get_graph();

  std::map<GroupedDependencyGraph::GroupGraph::vertex_descriptor, std::shared_ptr<ReactionGroup>> vertex_to_group;
  std::vector<std::shared_ptr<ReactionGroup>> all_groups;

  // create a reaction group for each vertex and keep track of all groups without dependencies in initial_groups_
  for (auto* vertex : boost::make_iterator_range(vertices(g))) {
    auto group = std::make_shared<ReactionGroup>();
    vertex_to_group[vertex] = group;
    all_groups.emplace_back(group);

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

  for (const auto& [_, group] : vertex_to_group) {
    auto& successors = group->successors;
    std::set<ReactionGroup*> in_super_group;
    if (successors.size() > 1) {
      for (auto* successor_a : successors) {
        if (in_super_group.count(successor_a) == 0 && successor_a->num_predecessors <= 1) {
          std::vector<ReactionGroup*> same_successors;
          std::copy_if(successors.begin(), successors.end(), std::back_insert_iterator(same_successors),
                       [successor_a](ReactionGroup* successor_b) {
                         return successor_b->num_predecessors <= 1 &&
                                successor_a->successors == successor_b->successors;
                       });
          if (same_successors.size() > 1) {
            auto super_group = std::make_shared<ReactionGroup>();
            all_groups.emplace_back(super_group);

            super_group->id = id_counter++;
            super_group->successors = successor_a->successors;
            super_group->num_predecessors = successor_a->num_predecessors;
            super_group->waiting_for.store(successor_a->num_predecessors, std::memory_order_release);
            super_group->triggered_sub_groups.resize(same_successors.size());

            for (auto* sub_group : same_successors) {
              super_group->sub_groups.emplace_back(sub_group);
              sub_group->super_group = super_group;

              // remove sub_group from the successor list of our starting group
              auto it = std::find(group->successors.begin(), group->successors.end(), sub_group);
              reactor_assert(it != group->successors.end());
              group->successors.erase(it);
            }
            // add the newly creates super_group as a successor to the starting group
            group->successors.emplace_back(super_group.get());

            log::Debug() << "Super Group for Group " << successor_a->id << ':';
            for (auto* elem : same_successors) {
              in_super_group.insert(elem);
              log::Debug() << " - Group " << elem->id;
            }
          }
        }
      }
    }
  }

  log::Debug() << "Identified reaction groups: ";
  for (const auto& group : all_groups) {
    if (group->sub_groups.empty()) {
      log::Debug() << "* Group " << group->id << ':';
      log::Debug() << "   + reactions:";
      for (auto [_, reaction] : group->reactions) {
        log::Debug() << "      - " << reaction->fqn();
      }
      if (group->super_group != nullptr) {
        log::Debug() << "   + super group: " << group->super_group->id;
      }
    } else {
      log::Debug() << "* Super Group " << group->id << ':';
      reactor_assert(group->reactions.empty());
      log::Debug() << "   + sub groups:";
      for (auto* sub_group : group->sub_groups) {
        log::Debug() << "      - " << sub_group->id;
      }
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

void GroupedSchedulingPolicy::schedule_until_ready_or_terminate(std::vector<ReactionGroup*>& ready_groups) {
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

auto GroupedSchedulingPolicy::finalize_group_and_notify_successors(ReactionGroup* group,
                                                                   std::vector<ReactionGroup*>& out_ready_groups)
    -> bool {
  group->waiting_for.store(group->num_predecessors, std::memory_order_release);
  notify_groups(group->successors, out_ready_groups);

  // return true if the group was the last to be processed.
  return 1 == groups_to_process_.fetch_sub(1, std::memory_order_acq_rel);
}

void GroupedSchedulingPolicy::notify_super_group(ReactionGroup* group, std::vector<ReactionGroup*>& out_ready_groups) {
  // the group is a super group with triggered sub groups
  // -> extract all the triggered subgroups
  std::atomic_thread_fence(std::memory_order_release);
  auto num_triggered = group->triggered_sub_groups_write_pos.load(std::memory_order_relaxed);
  for (std::size_t i{0}; i < num_triggered; i++) {
    out_ready_groups.emplace_back(group->triggered_sub_groups[i]);
    group->triggered_sub_groups[i]->triggered.store(false, std::memory_order_relaxed);
  }
  group->waiting_for.store(group->num_predecessors, std::memory_order_relaxed);
  group->triggered_sub_groups_write_pos.store(0, std::memory_order_relaxed);

  // we do not need to process the untriggered subgroups and can directly decrement the counter
  const auto num_untriggered = group->sub_groups.size() - num_triggered;
  if (num_untriggered > 0) {
    groups_to_process_.fetch_sub(num_untriggered, std::memory_order_acq_rel);
  }

  // update successor if there is any
  if (!group->successors.empty()) {
    reactor_assert(group->successors.size() == 1);
    reactor_assert(group->successors[0]->num_predecessors == 1);
    reactor_assert(group->successors[0]->sub_groups.empty());

    group->successors[0]->waiting_for.fetch_sub(num_untriggered, std::memory_order_acq_rel);
    // if none of the groups was triggered, then we can directly notify the successor
    if (num_triggered == 0) {
      notify_groups(group->successors, out_ready_groups);
    }
  }
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
      log::Debug() << "Group " << group->id << " is ready";
      if (group->triggered.exchange(false, std::memory_order_relaxed)) {
        out_ready_groups.emplace_back(group);
      } else if (!group->sub_groups.empty()) {
        notify_super_group(group, out_ready_groups);
      } else {
        finalize_group_and_notify_successors(group, out_ready_groups);
      }
    }
  }
  std::atomic_thread_fence(std::memory_order_acquire);
}

void GroupedSchedulingPolicy::terminate_workers() {
  group_queue_.reset();
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
    schedule_until_ready_or_terminate(ready_groups);
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
      schedule_until_ready_or_terminate(ready_groups);
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
  std::atomic_thread_fence(std::memory_order_release);
  bool old = group->triggered.exchange(true, std::memory_order_relaxed);
  // Also notify the super group if there is one and if we did not notify it yet.
  if (!old && group->super_group != nullptr) {
    log::Debug() << "(GroupedSchedulingPolicy) trigger group " << group->id << " in super group "
                 << group->super_group->id;
    auto pos = group->super_group->triggered_sub_groups_write_pos.fetch_add(1, std::memory_order_relaxed);
    group->super_group->triggered_sub_groups[pos] = group.get();
  }
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
