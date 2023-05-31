/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 *   Tassilo Tanneberger
 */

#include "reactor-cpp/environment.hh"

#include <algorithm>
#include <fstream>
#include <map>
#include <thread>
#include <vector>

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/statistics.hh"
#include "reactor-cpp/time.hh"

namespace reactor {

Environment::Environment(unsigned int num_workers, bool fast_fwd_execution, const Duration& timeout)
    : log_("Environment")
    , num_workers_(num_workers)
    , fast_fwd_execution_(fast_fwd_execution)
    , top_environment_(this)
    , scheduler_(this)
    , timeout_(timeout) {}

Environment::Environment(const std::string& name, Environment* containing_environment)
    : name_(name)
    , log_("Environment " + name)
    , num_workers_(containing_environment->num_workers_)
    , fast_fwd_execution_(containing_environment->fast_fwd_execution_)
    , containing_environment_(containing_environment)
    , top_environment_(containing_environment_->top_environment_)
    , scheduler_(this)
    , timeout_(containing_environment->timeout()) {
  [[maybe_unused]] bool result = containing_environment->contained_environments_.insert(this).second;
  reactor_assert(result);
}

void Environment::construct() {}

void Environment::insert_reactor(Reactor* reactor) {
  reactor_assert(reactor != nullptr);
  validate(this->phase() == Phase::Construction, "Reactors may only be registered during construction phase!");
  validate(reactor->is_top_level(), "The environment may only contain top level reactors!");
  [[maybe_unused]] bool result = top_level_reactors_.insert(reactor).second;
  reactor_assert(result);
}

void Environment::insert_input_action(BaseAction* action) {
  reactor_assert(action != nullptr);
  validate(this->phase() == Phase::Construction, "Input actions may only be registered during construction phase!");
  [[maybe_unused]] bool result = input_actions_.insert(action).second;
  reactor_assert(result);
  run_forever_ = true;
}

void Environment::sync_shutdown() {
  {
    std::lock_guard<std::mutex> lock{shutdown_mutex_};

    if (phase_ >= Phase::Shutdown) {
      // sync_shutdown() was already called -> abort
      return;
    }

    validate(phase_ == Phase::Execution, "sync_shutdown() may only be called during execution phase!");
    phase_ = Phase::Shutdown;
  }

  // the following will only be executed once
  log_.debug() << "Terminating the execution";

  for (auto* reactor : top_level_reactors_) {
    reactor->shutdown();
  }

  phase_ = Phase::Deconstruction;
  scheduler_.stop();
}

void Environment::async_shutdown() {
  auto lock_guard = scheduler_.lock();
  sync_shutdown();
}

auto Environment::startup() -> std::thread {
  validate(this == top_environment_, "startup() may only be called on the top environment");
  return startup(get_physical_time());
}

auto Environment::startup(const TimePoint& start_time) -> std::thread {
  validate(this->phase() == Phase::Assembly, "startup() may only be called during assembly phase!");

  log_.debug() << "Starting the execution";
  phase_ = Phase::Startup;

  this->start_tag_ = Tag::from_physical_time(start_time);
  // start up initialize all reactors

  for (auto* reactor : top_level_reactors_) {
    reactor->startup();
  }

  // start processing events
  phase_ = Phase::Execution;

  return std::thread([this, start_time]() {
    std::vector<std::thread> threads;
    threads.reserve(contained_environments_.size());
    // startup all contained environments recursively
    for (auto* env : contained_environments_) {
      threads.emplace_back(env->startup(start_time));
    }
    // start the local scheduler and wait until it returns
    this->scheduler_.start();

    // then join all the created threads
    for (auto& thread : threads) {
      thread.join();
    }
  });
}
void Environment::draw_connection(std::size_t source, std::size_t sink, ConnectionProperties properties) {
  graph_.add_edge(source, sink, properties);
}

auto Environment::register_port(BasePort* port) noexcept -> std::size_t {
  auto tag = ports_.size();
  port->set_tag(tag);
  ports_.push_back(port);
  return tag;
}

void recursive_assemble(Reactor* container) { // NOLINT
  container->assemble();
  for (auto* reactor : container->reactors()) {
    recursive_assemble(reactor);
  }
}
void Environment::assemble() {
  phase_ = Phase::Assembly;

  // constructing all the reactors
  // this mainly tell the reactors that they should connect their ports and actions not ports and ports
  for (auto* reactor : top_level_reactors_) {
    recursive_assemble(reactor);
  }

  auto graph = optimized_graph_.get_edges();
  // this generates the port graph
  for (auto const& [source, sinks] : graph) {

    auto source_port = source.first;
    auto properties = source.second;

    if (properties.type_ == ConnectionType::Normal) {
      for (const auto destination_port : sinks) {
        ports_[destination_port]->set_inward_binding(ports_[source_port]);
        ports_[source_port]->add_outward_binding(ports_[destination_port]);
        std::cout << "from: " << ports_[source_port]->container()->name() << "." << ports_[source_port]->name()
                  << " --> to: " << ports_[destination_port]->container()->name() << "."
                  << ports_[destination_port]->name() << std::endl;
      }
    } else {
      std::vector<BasePort*> pointers;
      std::transform(std::begin(sinks), std::end(sinks), std::back_inserter(pointers),
                     [this](std::size_t index) { return this->ports_[index]; });
      ports_[source_port]->pull_connection(properties, pointers);
    }
  }

  for (auto* reactor : top_level_reactors_) {
    build_dependency_graph(reactor);
  }

  calculate_indices();

  // this assembles all the contained environments aka enclaves
  for (auto* env : contained_environments_) {
    env->assemble();
  }
}

void Environment::optimize() {
  // TODO: do optimizations
  // TODO: maybe use the phases to guarantee that this is called before assemble
  optimized_graph_ = graph_;
}

void Environment::calculate_indices() {
  // build the graph
  std::map<Reaction*, std::set<Reaction*>> graph;
  for (auto* reaction : reactions_) {
    graph[reaction];
  }
  for (auto dependencies : dependencies_) {
    graph[dependencies.first].insert(dependencies.second);
  }

  log_.debug() << "Reactions sorted by index:";
  unsigned int index = 0;
  while (!graph.empty()) {
    // find nodes with degree zero and assign index
    std::set<Reaction*> degree_zero;
    for (auto& edge : graph) {
      if (edge.second.empty()) {
        edge.first->set_index(index);
        degree_zero.insert(edge.first);
      }
    }

    if (degree_zero.empty()) {
      // export_dependency_graph("/tmp/reactor_dependency_graph.dot");
      throw ValidationError("There is a loop in the dependency graph. Graph was written to "
                            "/tmp/reactor_dependency_graph.dot");
    }

    auto dbg = log_.debug();
    dbg << index << ": ";
    for (auto* reaction : degree_zero) {
      dbg << reaction->fqn() << ", ";
    }

    // reduce graph
    for (auto* reaction : degree_zero) {
      graph.erase(reaction);
    }
    for (auto& edge : graph) {
      for (auto* reaction : degree_zero) {
        edge.second.erase(reaction);
      }
    }

    index++;
  }

  max_reaction_index_ = index - 1;
}

void Environment::build_dependency_graph(Reactor* reactor) {
  // obtain dependencies from each contained reactor
  for (auto* sub_reactor : reactor->reactors()) {
    build_dependency_graph(sub_reactor);
  }
  // get reactions_ from this reactor; also order reactions_ by their priority
  std::map<int, Reaction*> priority_map;
  for (auto* reaction : reactor->reactions()) {
    reactions_.insert(reaction);
    auto result = priority_map.emplace(reaction->priority(), reaction);
    validate(result.second, "priorities must be unique for all reactions_ of the same reactor");
  }

  // connect all reactions_ this reaction depends on
  for (auto* reaction : reactor->reactions()) {
    for (auto* dependency : reaction->dependencies()) {
      auto* source = dependency;
      while (source->has_inward_binding()) {
        source = source->untyped_inward_binding();
      }
      for (auto* antidependency : source->anti_dependencies()) {
        dependencies_.emplace_back(reaction, antidependency);
      }
    }
  }

  // connect reactions_ by priority
  if (priority_map.size() > 1) {
    auto iterator = priority_map.begin();
    auto next = std::next(iterator);
    while (next != priority_map.end()) {
      dependencies_.emplace_back(next->second, iterator->second);
      iterator++;
      next = std::next(iterator);
    }
  }
}

} // namespace reactor
