/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/environment.hh"

#include <cassert>

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
}  // namespace dear
