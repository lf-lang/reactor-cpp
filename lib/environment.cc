/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/environment.hh"
#include "dear/logging.hh"
#include "dear/port.hh"
#include "dear/reaction.hh"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <map>

namespace dear {

void Environment::register_reactor(Reactor* reactor) {
  assert(this->phase() == Phase::Construction);
  assert(reactor != nullptr);
  assert(reactor->is_top_level());
  auto r = _top_level_reactors.insert(reactor);
  assert(r.second);
}

void recursive_assemble(Reactor* container) {
  container->assemble();
  for (auto r : container->reactors()) {
    recursive_assemble(r);
  }
}

void Environment::assemble() {
  assert(_phase == Phase::Construction);
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
    assert(result.second && "priorities must be unique (for now)");
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
  auto it = priority_map.begin();
  auto next = std::next(it);
  while (next != priority_map.end()) {
    dependencies.push_back(std::make_pair(next->second, it->second));
    it++;
    next = std::next(it);
  }
}

void Environment::init() {
  log::Info() << "Initializing the environment";
  // build the dependency graph
  for (auto r : _top_level_reactors) {
    build_dependency_graph(r);
  }
  calculate_indexes();
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

}  // namespace dear
