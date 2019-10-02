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

void Timer::init(const Tag& t0) {
  if (_offset != 0) {
    Tag t1 = t0.delay(_offset);
    environment()->scheduler()->schedule(t1, this, [this]() { reschedule(); });
  } else {
    environment()->scheduler()->schedule(t0, this, [this]() { reschedule(); });
  }
}

void Timer::reschedule() {
  if (_period != 0) {
    Tag now = Tag::from_logical_time(environment()->logical_time());
    Tag next = now.delay(_period);
    environment()->scheduler()->schedule(next, this,
                                         [this]() { reschedule(); });
  }
}

}  // namespace dear
