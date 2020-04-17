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

void BaseAction::register_trigger(Reaction* reaction) {
  ASSERT(reaction != nullptr);
  ASSERT(this->environment() == reaction->environment());
  VALIDATE(this->environment()->phase() == Environment::Phase::Assembly,
           "Triggers may only be registered during assembly phase!");
  VALIDATE(this->container() == reaction->container(),
           "Action triggers must belong to the same reactor as the triggered "
           "reaction");
  auto r = _triggers.insert(reaction);
  ASSERT(r.second);
}

void BaseAction::register_scheduler(Reaction* reaction) {
  ASSERT(reaction != nullptr);
  ASSERT(this->environment() == reaction->environment());
  VALIDATE(is_logical(), "only logical action can be scheduled by a reaction!");
  VALIDATE(
      this->environment()->phase() == Environment::Phase::Assembly,
      "Schedulers for actions may only be registered during assembly phase!");
  // the reaction must belong to the same reactor as this action
  VALIDATE(this->container() == reaction->container(),
           "Scheduable actions must belong to the same reactor as the "
           "triggered reaction");
  auto r = _schedulers.insert(reaction);
  ASSERT(r.second);
}

void Timer::startup() {
  Tag t0 = Tag::from_physical_time(environment()->start_time());
  if (_offset != Duration::zero()) {
    environment()->scheduler()->schedule_sync(t0.delay(_offset), this, nullptr);
  } else {
    environment()->scheduler()->schedule_sync(t0, this, nullptr);
  }
}

void Timer::cleanup() {
  // schedule the timer again
  if (_period != Duration::zero()) {
    Tag now = Tag::from_logical_time(environment()->logical_time());
    Tag next = now.delay(_period);
    environment()->scheduler()->schedule_sync(next, this, nullptr);
  }
}

void ShutdownAction::shutdown() {
  Tag t = Tag::from_logical_time(environment()->logical_time()).delay();
  environment()->scheduler()->schedule_sync(t, this, nullptr);
}

}  // namespace reactor
