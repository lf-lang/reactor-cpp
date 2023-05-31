/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_IMPL_PORT_IMPL_HH
#define REACTOR_CPP_IMPL_PORT_IMPL_HH

#include "reactor-cpp/assert.hh"
#include "reactor-cpp/connection.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/port.hh"

namespace reactor {

template<class T>
class DelayedConnection;

template<class T>
class PhysicalConnection;

template <class T> [[maybe_unused]] auto Port<T>::typed_outward_bindings() const noexcept -> const std::set<Port<T>*>& {
  return outward_bindings_; // NOLINT C++20 std::bit_cast
}

template <class T> auto Port<T>::typed_inward_binding() const noexcept -> Port<T>* {
  // we can use a static cast here since we know that this port is always
  // connected with another Port<T>.
  return inward_binding_;
}

template <class T> void Port<T>::set(const ImmutableValuePtr<T>& value_ptr) {
  validate(!has_inward_binding(), "set() may only be called on ports that do not have an inward "
                                  "binding!");
  validate(value_ptr != nullptr, "Ports may not be set to nullptr!");

  this->present_ = true;
  this->value_ptr_ = std::move(value_ptr);

  this->invoke_set_callback();

  for (auto* const outward : outward_bindings_) {
    outward->set(value_ptr);
  };

  auto* scheduler = environment()->scheduler();
  // this is the start of a crime scene further investigation may lead to psychological terror
  // we insert here everything in batches to reduce how often the env needs to be loaded from main memory
  // when every port would insert itself individually
  if (!outward_bindings_.empty()) {
    scheduler->set_ports<reactor::Port<T>>(std::end(outward_bindings_), std::end(outward_bindings_));
  }
  if (!triggers().empty()) {
    scheduler->set_triggers(std::begin(triggers()), std::end(triggers()));
  }
}

template <class T> auto Port<T>::get() const noexcept -> const ImmutableValuePtr<T>& {
  if (has_inward_binding()) {
    return typed_inward_binding()->get();
  }
  return value_ptr_;
}

template <class T>
auto Port<T>::pull_connection(ConnectionProperties* properties) -> Connection<T>*  {
  if (properties->type_ == ConnectionType::Delayed) {
    return new DelayedConnection<T>{this->name() + "_delayed_connection", this->container(), properties->delay_};
  }
  if (properties->type_ == ConnectionType::Physical) {
    return new PhysicalConnection<T>{this->name() + "_physical_connection", this->container(), properties->delay_};
  }

  return nullptr;
}

} // namespace reactor

#endif // REACTOR_CPP_IMPL_PORT_IMPL_HH
