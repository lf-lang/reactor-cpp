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

#include "reactor.hh"
#include "scheduler.hh"

namespace reactor {

class Environment {
 public:
  enum class Phase {
    Construction,
    Assembly,
    Startup,
    Execution,
    Shutdown,
    Deconstruction
  };

 private:
  const unsigned _num_workers;
  Scheduler _scheduler;
  const bool _fast_fwd_execution;
  std::set<Reactor*> _top_level_reactors;

  std::set<Reaction*> reactions;
  using Dependency = std::pair<Reaction*, Reaction*>;
  std::vector<Dependency> dependencies;
  Phase _phase{Phase::Construction};

  std::map<Reaction*, unsigned> indexes;

  void build_dependency_graph(Reactor* reactor);
  void calculate_indexes();

  time_t _start_time;

 public:
  Environment(unsigned num_workers, bool fast_fwd_execution = false)
      : _num_workers(num_workers)
      , _scheduler(this)
      , _fast_fwd_execution(fast_fwd_execution) {}

  void register_reactor(Reactor* reactor);

  const auto& top_level_reactors() const { return _top_level_reactors; }

  void assemble();
  std::thread startup();
  void sync_shutdown();
  void async_shutdown();

  void export_dependency_graph(const std::string& path);

  Phase phase() const { return _phase; }
  const Scheduler* scheduler() const { return &_scheduler; }
  Scheduler* scheduler() { return &_scheduler; }

  unsigned get_index(Reaction* r) const { return indexes.at(r); }

  const LogicalTime& logical_time() const { return _scheduler.logical_time(); }
  time_t start_time() const { return _start_time; }

  unsigned num_workers() const { return _num_workers; }
  bool fast_fwd_execution() const { return _fast_fwd_execution; }
};

}  // namespace reactor
