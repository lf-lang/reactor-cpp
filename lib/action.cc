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
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);
  // the reaction must belong to the same reactor as this action
  ASSERT(this->container() == reaction->container());
  auto r = _triggers.insert(reaction);
  ASSERT(r.second);
}

void BaseAction::register_scheduler(Reaction* reaction) {
  ASSERT(reaction != nullptr);
  ASSERT(is_logical());
  ASSERT(this->environment() == reaction->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);
  // the reaction must belong to the same reactor as this action
  ASSERT(this->container() == reaction->container());
  auto r = _schedulers.insert(reaction);
  ASSERT(r.second);
}

void Action<void>::schedule(time_t delay) {
  auto scheduler = environment()->scheduler();
  auto setup = [this]() { this->present = true; };
  if (is_logical()) {
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(delay);
    scheduler->schedule(tag, this, setup);
  } else {
    auto tag = Tag::from_physical_time(delay);
    scheduler->schedule(tag, this, setup);
  }
}

void Timer::init(const Tag& t0) {
  if (_offset != 0) {
    Tag t1 = t0.delay(_offset);
    environment()->scheduler()->schedule(t1, this, nullptr);
  } else {
    environment()->scheduler()->schedule(t0, this, nullptr);
  }
}

void Timer::cleanup() {
  // schedule the timer again
  if (_period != 0) {
    Tag now = Tag::from_logical_time(environment()->logical_time());
    Tag next = now.delay(_period);
    environment()->scheduler()->schedule(next, this, nullptr);
  }
}

}  // namespace reactor
