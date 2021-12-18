/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "../assert.hh"
#include "../environment.hh"

namespace reactor {

template <class T>
template <class Dur>
void Action<T>::schedule(const ImmutableValuePtr<T>& value_ptr, Dur delay) {
  auto d = std::chrono::duration_cast<Duration>(delay);
  reactor::validate(d >= Duration::zero(),
           "Schedule cannot be called with a negative delay!");
  reactor::validate(value_ptr != nullptr,
           "Actions may not be scheduled with a nullptr value!");
  auto scheduler = environment()->scheduler();
  auto setup = [value_ptr, this]() { this->value_ptr_ = std::move(value_ptr); };
  if (is_logical()) {
    d += this->min_delay;
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(d);
    scheduler->schedule_sync(tag, this, setup);
  } else {
    auto tag = Tag::from_physical_time(get_physical_time() + d);
    scheduler->schedule_async(tag, this, setup);
  }
}

template <class Dur>
void Action<void>::schedule(Dur delay) noexcept {
  auto d = std::chrono::duration_cast<Duration>(delay);
  reactor::validate(d >= Duration::zero(),
           "Schedule cannot be called with a negative delay!");
  auto scheduler = environment()->scheduler();
  auto setup = [this]() { this->present_ = true; };
  if (is_logical()) {
    d += this->min_delay;
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(d);
    scheduler->schedule_sync(tag, this, setup);
  } else {
    // physical action
    auto tag = Tag::from_physical_time(get_physical_time() + d);
    scheduler->schedule_async(tag, this, setup);
  }
}

}  // namespace reactor
