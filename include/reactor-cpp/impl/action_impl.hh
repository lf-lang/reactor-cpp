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
#include <iterator>
#include <mutex>

namespace reactor {

template <class T> template <class Dur> void Action<T>::schedule(const ImmutableValuePtr<T>& value_ptr, Dur delay) {
  Duration time_delay = std::chrono::duration_cast<Duration>(delay);
  reactor::validate(time_delay >= Duration::zero(), "Schedule cannot be called with a negative delay!");
  reactor::validate(value_ptr != nullptr, "Actions may not be scheduled with a nullptr value!");
  auto* scheduler = environment()->scheduler();
  if (is_logical()) {
    time_delay += this->min_delay();
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(time_delay); // NOLINT
    events_[tag] = value_ptr;
    scheduler->schedule_sync(tag, this);
  } else {
    auto tag = Tag::from_physical_time(get_physical_time() + time_delay);
    {
      std::lock_guard<std::mutex> lock(mutex_events_);
      events_[tag] = value_ptr;
    }
    scheduler->schedule_async(tag, this);
  }
}

template <class Dur> void Action<void>::schedule(Dur delay) {
  auto time_delay = std::chrono::duration_cast<Duration>(delay);
  reactor::validate(time_delay >= Duration::zero(), "Schedule cannot be called with a negative delay!");
  auto* scheduler = environment()->scheduler();
  if (is_logical()) {
    time_delay += this->min_delay();
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(time_delay);
    scheduler->schedule_sync(tag, this);
  } else {
    // physical action
    auto tag = Tag::from_physical_time(get_physical_time() + time_delay);
    scheduler->schedule_async(tag, this);
  }
}

template <class T> void Action<T>::setup() noexcept {
  if (value_ptr_ == nullptr) {
    const auto& node = events_.extract(events_.begin());
    reactor_assert(!node.empty());
    value_ptr_ = std::move(node.mapped());
  }
}

} // namespace reactor

#endif
