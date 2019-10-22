/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "dear/reactor.hh"
#include "dear/scheduler.hh"

namespace dear {

class Environment {
 public:
  enum class Phase { Construction, Assembly, Initialization, Execution };

 private:
  Scheduler _scheduler;
  std::set<Reactor*> _top_level_reactors;

  std::set<Reaction*> reactions;
  using Dependency = std::pair<Reaction*, Reaction*>;
  std::vector<Dependency> dependencies;
  Phase _phase{Phase::Construction};

  std::map<Reaction*, unsigned> indexes;

  void build_dependency_graph(Reactor* reactor);
  void calculate_indexes();

 public:
  Environment(unsigned num_workers) : _scheduler(this, num_workers) {}

  void register_reactor(Reactor* reactor);

  const auto& top_level_reactors() const { return _top_level_reactors; }

  void assemble();
  void init();
  std::thread start();

  void export_dependency_graph(const std::string& path);

  Phase phase() const { return _phase; }
  const Scheduler* scheduler() const { return &_scheduler; }
  Scheduler* scheduler() { return &_scheduler; }

  unsigned get_index(Reaction* r) const { return indexes.at(r); }

  const LogicalTime& logical_time() const { return _scheduler.logical_time(); }
};

}  // namespace dear
