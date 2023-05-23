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
#include "reactor-cpp/time.hh"

namespace reactor {

void BaseAction::register_trigger(Reaction* reaction) {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment());
  assert_phase(this, Phase::Assembly);
  validate(this->container() == reaction->container(),
           "Action triggers must belong to the same reactor as the triggered "
           "reaction");
  [[maybe_unused]] bool result = triggers_.insert(reaction).second;
  reactor_assert(result);
}

void BaseAction::register_scheduler(Reaction* reaction) {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment());
  assert_phase(this, Phase::Assembly);
  // the reaction must belong to the same reactor as this action
  validate(this->container() == reaction->container(), "Scheduable actions must belong to the same reactor as the "
                                                       "triggered reaction");
  [[maybe_unused]] bool result = schedulers_.insert(reaction).second;
  reactor_assert(result);
}

void Timer::startup() {
  // abort if the offset is the maximum duration
  if (offset_ == Duration::max()) {
    return;
  }

  const Tag& start_tag = environment()->start_tag();
  if (offset_ != Duration::zero()) {
    environment()->scheduler()->schedule_sync(this, start_tag.delay(offset_));
  } else {
    environment()->scheduler()->schedule_sync(this, start_tag);
  }
}

void Timer::cleanup() noexcept {
  BaseAction::cleanup();
  // schedule the timer again
  if (period_ != Duration::zero()) {
    Tag now = Tag::from_logical_time(environment()->logical_time());
    Tag next = now.delay(period_);
    environment()->scheduler()->schedule_sync(this, next);
  }
}

ShutdownTrigger::ShutdownTrigger(const std::string& name, Reactor* container)
    : Timer(name, container, Duration::zero(), container->environment()->timeout()) {}

void ShutdownTrigger::setup() noexcept {
  BaseAction::setup();
  environment()->sync_shutdown();
}

void ShutdownTrigger::shutdown() {
  if (!is_present()) {
    Tag tag = Tag::from_logical_time(environment()->logical_time()).delay();
    environment()->scheduler()->schedule_sync(this, tag);
  }
}

auto Action<void>::schedule_at(const Tag& tag) -> bool {
  auto* scheduler = environment()->scheduler();
  if (is_logical()) {
    if (tag <= scheduler->logical_time()) {
      return false;
    }
    scheduler->schedule_sync(this, tag);
  } else {
    // We must call schedule_async while holding the mutex, because otherwise
    // the scheduler could already start processing the event that we schedule
    // and call setup() on this action before we insert the value in events_.
    // Holding both the local mutex mutex_events_ and the scheduler mutex (in
    // schedule async) should not lead to a deadlock as the scheduler will
    // only hold one of the two mutexes at once.
    return scheduler->schedule_async_at(this, tag);
  }
  return true;
}

} // namespace reactor
