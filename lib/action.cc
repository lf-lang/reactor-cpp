/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/action.hh"

#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/reaction.hh"

namespace reactor {

void BaseAction::register_trigger(Reaction *reaction) {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment());
  assert_phase(this, Environment::Phase::Assembly);
  validate(this->container() == reaction->container(),
           "Action triggers must belong to the same reactor as the triggered "
           "reaction");
  reactor_assert(triggers_.insert(reaction).second);
}

void BaseAction::register_scheduler(Reaction *reaction) {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment());
  validate(is_logical(), "only logical action can be scheduled by a reaction!");
  assert_phase(this, Environment::Phase::Assembly);
  // the reaction must belong to the same reactor as this action
  validate(this->container() == reaction->container(),
           "Scheduable actions must belong to the same reactor as the "
           "triggered reaction");
  reactor_assert(schedulers_.insert(reaction).second);
}

void Timer::startup() {
  Tag tag_zero = Tag::from_physical_time(environment()->start_time());
  if (offset_ != Duration::zero()) {
    environment()->scheduler()->schedule_sync(tag_zero.delay(offset_), this, nullptr);
  } else {
    environment()->scheduler()->schedule_sync(tag_zero, this, nullptr);
  }
}

void Timer::cleanup() {
  // schedule the timer again
  if (period_ != Duration::zero()) {
    Tag now = Tag::from_logical_time(environment()->logical_time());
    Tag next = now.delay(period_);
    environment()->scheduler()->schedule_sync(next, this, nullptr);
  }
}

void ShutdownAction::shutdown() {
  Tag tag = Tag::from_logical_time(environment()->logical_time()).delay();
  environment()->scheduler()->schedule_sync(tag, this, nullptr);
}

} // namespace reactor
