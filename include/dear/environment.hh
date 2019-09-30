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

namespace dear {

class Environment {
 public:
  enum class Phase { Construction, Assembly };

 private:
  std::set<Reactor*> _top_level_reactors;

  std::set<Reaction*> reactions;
  using Dependency = std::pair<Reaction*, Reaction*>;
  std::vector<Dependency> dependencies;
  Phase _phase{Phase::Construction};

  std::map<Reaction*, unsigned> indexes;

  void build_dependency_graph(Reactor* reactor);
  void calculate_indexes();

 public:
  Environment() = default;

  void register_reactor(Reactor* reactor);

  const auto& top_level_reactors() const { return _top_level_reactors; }

  void assemble();
  void init();
  void export_dependency_graph(const std::string& path);

  Phase phase() const { return _phase; }
};

}  // namespace dear
