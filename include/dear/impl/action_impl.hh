/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "../assert.hh"
#include "../environment.hh"

namespace dear {

template <class T>
void Action<T>::schedule(const ImmutableValuePtr<T>& value_ptr, time_t delay) {
  ASSERT(value_ptr != nullptr);
  auto scheduler = environment()->scheduler();
  auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(delay);
  scheduler->schedule(tag, this, [value_ptr, this]() {
    this->value_ptr = std::move(value_ptr);
  });
}

}  // namespace dear
