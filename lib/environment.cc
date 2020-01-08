/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/environment.hh"

#include <algorithm>
#include <fstream>
#include <map>

#include "reactor-cpp/assert.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/validate.hh"

namespace reactor {

void Environment::register_reactor(Reactor* reactor) {
  ASSERT(reactor != nullptr);
  validate(this->phase() == Phase::Construction,
           "Reactors may only be registered during construction phase!");
  validate(reactor->is_top_level(),
           "The environment may only contain top level reactors!");
  auto r = _top_level_reactors.insert(reactor);
  ASSERT(r.second);
}

void recursive_assemble(Reactor* container) {
  container->assemble();
  for (auto r : container->reactors()) {
    recursive_assemble(r);
  }
}

void Environment::assemble() {
  validate(this->phase() == Phase::Construction,
           "assemble() may only be called during construction phase!");
  _phase = Phase::Assembly;
  for (auto r : _top_level_reactors) {
    recursive_assemble(r);
  }
}

void Environment::build_dependency_graph(Reactor* reactor) {
  // obtain dependencies from each contained reactor
  for (auto r : reactor->reactors()) {
    build_dependency_graph(r);
  }
  // get reactions from this reactor; also order reactions by their priority
  std::map<int, Reaction*> priority_map;
  for (auto r : reactor->reactions()) {
    reactions.insert(r);
    auto result = priority_map.emplace(r->priority(), r);
    validate(result.second,
             "priorities must be unique for all reactions of the same reactor");
  }

  // connect all reactions this reaction depends on
  for (auto r : reactor->reactions()) {
    for (auto d : r->dependencies()) {
      auto source = d;
      while (source->has_inward_binding()) {
        source = source->inward_binding();
      }
      for (auto ad : source->antidependencies()) {
        dependencies.push_back(std::make_pair(r, ad));
      }
    }
  }

  // connect reactions by priority
  if (priority_map.size() > 1) {
    auto it = priority_map.begin();
    auto next = std::next(it);
    while (next != priority_map.end()) {
      dependencies.push_back(std::make_pair(next->second, it->second));
      it++;
      next = std::next(it);
    }
  }
}

std::thread Environment::startup() {
  validate(this->phase() == Phase::Assembly,
           "startup() may only be called during assembly phase!");

  // build the dependency graph
  for (auto r : _top_level_reactors) {
    build_dependency_graph(r);
  }
  calculate_indexes();

  log::Info() << "Starting the execution";
  _phase = Phase::Startup;

  _start_time = get_physical_time();
  // startupialize all reactors
  for (auto r : _top_level_reactors) {
    r->startup();
  }

  // start processing events
  _phase = Phase::Execution;
  return std::thread([this]() { this->_scheduler.start(); });
}

void Environment::sync_shutdown() {
  validate(this->phase() == Phase::Execution,
           "sync_shutdown() may only be called during execution phase!");
  _phase = Phase::Shutdown;

  log::Info() << "Terminating the execution";

  for (auto r : _top_level_reactors) {
    r->shutdown();
  }

  _phase = Phase::Deconstruction;

  _scheduler.stop();
}

void Environment::async_shutdown() {
  _scheduler.lock();
  sync_shutdown();
  _scheduler.unlock();
}

std::string dot_name(ReactorElement* r) {
  std::string fqn = r->fqn();
  std::replace(fqn.begin(), fqn.end(), '.', '_');
  return fqn;
}

void Environment::export_dependency_graph(const std::string& path) {
  std::ofstream dot;
  dot.open(path);

  dot << "digraph {\n";
  for (auto r : reactions) {
    dot << dot_name(r) << " [label=\"" << r->fqn() << "\"];" << std::endl;
  }
  for (auto d : dependencies) {
    dot << dot_name(d.first) << " -> " << dot_name(d.second) << ';'
        << std::endl;
  }
  dot << "}\n";

  dot.close();
}

void Environment::calculate_indexes() {
  // build the graph
  std::map<Reaction*, std::set<Reaction*>> graph;
  for (auto r : reactions) {
    graph[r];
  }
  for (auto d : dependencies) {
    graph[d.first].insert(d.second);
  }

  log::Debug() << "Reactions sorted by index:";
  unsigned index = 0;
  while (graph.size() != 0) {
    // find nodes with degree zero and assign index
    std::set<Reaction*> degree_zero;
    for (auto& kv : graph) {
      if (kv.second.size() == 0) {
        indexes[kv.first] = index;
        degree_zero.insert(kv.first);
      }
    }

    log::Debug dbg;
    dbg << index << ": ";
    for (auto r : degree_zero) {
      dbg << r->fqn() << ", ";
    }

    // reduce graph
    for (auto r : degree_zero) {
      graph.erase(r);
    }
    for (auto& kv : graph) {
      for (auto r : degree_zero) {
        kv.second.erase(r);
      }
    }

    index++;
  }
}

}  // namespace reactor
