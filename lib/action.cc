/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/action.hh"
#include "dear/environment.hh"
#include "dear/reaction.hh"

#include <cassert>

namespace dear {

void BaseAction::register_trigger(Reaction* reaction) {
  assert(reaction != nullptr);
  assert(this->environment() == reaction->environment());
  assert(this->environment()->phase() == Environment::Phase::Assembly);
  // the reaction must belong to the same reactor as this action
  assert(this->container() == reaction->container());
  auto r = _triggers.insert(reaction);
  assert(r.second);
}

void BaseAction::register_scheduler(Reaction* reaction) {
  assert(reaction != nullptr);
  assert(this->environment() == reaction->environment());
  assert(this->environment()->phase() == Environment::Phase::Assembly);
  // the reaction must belong to the same reactor as this action
  assert(this->container() == reaction->container());
  auto r = _triggers.insert(reaction);
  assert(r.second);
}

}  // namespace dear
