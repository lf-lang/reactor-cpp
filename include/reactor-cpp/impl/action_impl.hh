/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_IMPL_ACTION_IMPL_HH
#define REACTOR_CPP_IMPL_ACTION_IMPL_HH

#include "../assert.hh"
#include "../environment.hh"

namespace reactor {

template <class T> template <class Dur> void Action<T>::schedule(const ImmutableValuePtr<T>& value_ptr, Dur delay) {
  Duration time_delay = std::chrono::duration_cast<Duration>(delay); // NOLINT
  reactor::validate(time_delay >= Duration::zero(), "Schedule cannot be called with a negative delay!");
  reactor::validate(value_ptr != nullptr, "Actions may not be scheduled with a nullptr value!");
  auto* scheduler = environment()->scheduler();                                  // NOLINT
  auto setup = [value_ptr, this]() { this->value_ptr_ = std::move(value_ptr); }; // NOLINT
  if (is_logical()) {
    time_delay += this->min_delay();
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(time_delay); // NOLINT
    scheduler->schedule_sync(tag, this, setup);
  } else {
    auto tag = Tag::from_physical_time(get_physical_time() + time_delay); // NOLINT
    scheduler->schedule_async(tag, this, setup);
  }
}

template <class Dur> void Action<void>::schedule(Dur delay) {
  auto time_delay = std::chrono::duration_cast<Duration>(delay); // NOLINT
  reactor::validate(time_delay >= Duration::zero(), "Schedule cannot be called with a negative delay!");
  auto* scheduler = environment()->scheduler();     // NOLINT
  auto setup = [this]() { this->present_ = true; }; // NOLINT
  if (is_logical()) {
    time_delay += this->min_delay();
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(time_delay); // NOLINT
    scheduler->schedule_sync(tag, this, setup);
  } else {
    // physical action
    auto tag = Tag::from_physical_time(get_physical_time() + time_delay); // NOLINT
    scheduler->schedule_async(tag, this, setup);
  }
}

} // namespace reactor

#endif
