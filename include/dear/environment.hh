/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <set>
#include <string>

#include "dear/reactor.hh"

namespace dear {

class Environment {
 private:
  std::set<Reactor*> _top_level_reactors;

 public:
  Environment(){};

  void register_reactor(Reactor* reactor);

  const auto& top_level_reactors() const { return _top_level_reactors; }

  void export_dependency_graph(const std::string& path);
};

}  // namespace dear
