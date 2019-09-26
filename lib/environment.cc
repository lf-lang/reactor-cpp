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
  assert(reactor != nullptr);
  assert(reactor->is_top_level());
  auto r = _top_level_reactors.insert(reactor);
  assert(r.second);
}

}  // namespace dear
