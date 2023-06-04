/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/port.hh"

#include "reactor-cpp/assert.hh"
#include "reactor-cpp/connection.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/statistics.hh"

namespace reactor {

void BasePort::register_dependency(Reaction* reaction, bool is_trigger) noexcept {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment());
  assert_phase(this, Phase::Assembly);

  if (this->is_input()) {
    validate(this->container() == reaction->container(), "Dependent input ports must belong to the same reactor as the "
                                                         "reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Dependent output ports must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = dependencies_.insert(reaction).second;
  reactor_assert(result);
  if (is_trigger) {
    result = triggers_.insert(reaction).second;
    reactor_assert(result);
  }
}

void BasePort::register_antidependency(Reaction* reaction) noexcept {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment());
  validate(!this->has_inward_binding(), "Antidependencies may no be declared on ports with an inward binding!");
  assert_phase(this, Phase::Assembly);

  if (this->is_output()) {
    validate(this->container() == reaction->container(),
             "Antidependent output ports must belong to the same reactor as "
             "the reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

  [[maybe_unused]] bool result = anti_dependencies_.insert(reaction).second;
  reactor_assert(result);
}

[[maybe_unused]] auto Port<void>::typed_outward_bindings() const noexcept -> const std::set<Port<void>*>& {
  // this is undefined behavior and should be changed
  return reinterpret_cast<const std::set<Port<void>*>&>(outward_bindings_); // NOLINT C++20 std::bit_cast
}

auto Port<void>::typed_inward_binding() const noexcept -> Port<void>* {
  // we can use a reinterpret cast here since we know that this port is always
  // connected with another Port<T>.
  return reinterpret_cast<Port<void>*>(inward_binding_); // NOLINT
}

void Port<void>::set() {
  validate(!has_inward_binding(), "set() may only be called on ports that do not have an inward "
                                  "binding!");

  this->present_ = true;

  this->invoke_set_callback();

  for (auto* const outward : outward_bindings_) {
    outward->set();
  };

  auto* scheduler = environment()->scheduler();
  // this is the start of a crime scene further investigation may lead to psychological terror
  // we insert here everything in batches to reduce how often the env needs to be loaded from main memory
  // when every port would insert itself individually
  if (!outward_bindings_.empty()) {
    scheduler->set_ports<reactor::Port<void>>(std::end(outward_bindings_), std::end(outward_bindings_));
  }
  if (!triggers().empty()) {
    scheduler->set_triggers(std::begin(triggers()), std::end(triggers()));
  }
}

void Port<void>::pull_connection(const ConnectionProperties& properties, const std::vector<BasePort*>& downstream) {
  Connection<void>* connection = nullptr;
  if (properties.type_ == ConnectionType::Delayed) {
    connection =
        new DelayedConnection<void>(this->name() + "_delayed_connection", this->container(), properties.delay_);
  }
  if (properties.type_ == ConnectionType::Physical) {
    connection =
        new PhysicalConnection<void>(this->name() + "_physical_connection", this->container(), properties.delay_);
  }

  if (connection != nullptr) {
    connection->bind_downstream_ports(downstream);
    connection->bind_upstream_port(this);
    this->register_set_callback(connection->upstream_set_callback());
    this->container()->register_connection(connection);
  }
}

// This function can be used to chain two callbacks. This mechanism is not
// very efficient if many callbacks are registered on the same port. However,
// it is more efficient than, e.g., a vector of callbacks if usually only one
// callback is registered. At the moment, we use at most two callbacks on the
// same port (one if the port is in a multiport, and one if it is upstream of
// a delayed connection).
auto compose_callbacks(const PortCallback& callback1, const PortCallback& callback2) -> PortCallback {
  return [=](const BasePort& port) {
    callback1(port);
    callback2(port);
  };
}

void BasePort::register_set_callback(const PortCallback& callback) {
  if (set_callback_ == nullptr) {
    set_callback_ = callback;
  } else {
    set_callback_ = compose_callbacks(set_callback_, callback);
  }
}

void BasePort::register_clean_callback(const PortCallback& callback) {
  if (clean_callback_ == nullptr) {
    clean_callback_ = callback;
  } else {
    clean_callback_ = compose_callbacks(clean_callback_, callback);
  }
}

} // namespace reactor
