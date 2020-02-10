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
  validate(d >= Duration::zero(),
           "Schedule cannot be called with a negative delay!");
  ASSERT(value_ptr != nullptr);
  auto scheduler = environment()->scheduler();
  auto setup = [value_ptr, this]() { this->value_ptr = std::move(value_ptr); };
  if (is_logical()) {
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(d);
    scheduler->schedule(tag, this, setup);
  } else {
    auto tag = Tag::from_physical_time(get_physical_time() + d);
    scheduler->schedule(tag, this, setup);
  }
}

template <class Dur>
void Action<void>::schedule(Dur delay) {
  auto d = std::chrono::duration_cast<Duration>(delay);
  validate(d >= Duration::zero(),
           "Schedule cannot be called with a negative delay!");
  auto scheduler = environment()->scheduler();
  auto setup = [this]() { this->present = true; };
  if (is_logical()) {
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(d);
    scheduler->schedule(tag, this, setup);
  } else {
    // physical action
    scheduler->lock();
    auto tag = Tag::from_physical_time(get_physical_time() + d);
    scheduler->schedule(tag, this, setup);
    scheduler->unlock();
  }
}

}  // namespace reactor
