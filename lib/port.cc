/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/port.hh"

#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/reaction.hh"

namespace reactor {

void BasePort::base_bind_to(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor_assert(this->environment() == port->environment()); // NOLINT
  validate(!port->has_inward_binding(), "Ports may only be connected once");
  validate(!port->has_anti_dependencies(), "Ports with anti dependencies may not be connected to other ports");
  assert_phase(this, Environment::Phase::Assembly);
  if (this->is_input() && port->is_input()) {
    validate(this->container() == port->container()->container(),
             "An input port A may only be bound to another input port B if B is contained by a reactor that in turn is "
             "contained by the reactor of A");
  } else if (this->is_input() && port->is_output()) {
    validate(
        this->container() == port->container(),
        "An input port A may only be bound directly to an output port B if A and B are contained by the same reactor.");
  } else if (this->is_output() && port->is_input()) {
    validate(this->container()->container() == port->container()->container(),
             "An output port can only be bound to an input port if both ports belong to reactors in the same "
             "hierarichal level");
  } else if (this->is_output() && port->is_output()) {
    validate(this->container()->container() == port->container(),
             "An output port A may only be bound to another output port B if A is contained by a reactor that in turn "
             "is contained by the reactor of B");
  } else {
    throw std::runtime_error("invalid connection");
  }

  port->inward_binding_ = this;
  reactor_assert(this->outward_bindings_.insert(port).second);
}

void BasePort::register_dependency(Reaction* reaction, bool is_trigger) noexcept {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment()); // NOLINT
  validate(!this->has_outward_bindings(), "Dependencies may no be declared on ports with an outward binding!");
  assert_phase(this, Environment::Phase::Assembly);

  if (this->is_input()) {
    validate(this->container() == reaction->container(), "Dependent input ports must belong to the same reactor as the "
                                                         "reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Dependent output ports must belong to a contained reactor");
  }

  reactor_assert(dependencies_.insert(reaction).second);
  if (is_trigger) {
    reactor_assert(triggers_.insert(reaction).second);
  }
}

void BasePort::register_antidependency(Reaction* reaction) noexcept {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment()); // NOLINT
  validate(!this->has_inward_binding(), "Antidependencies may no be declared on ports with an inward binding!");
  assert_phase(this, Environment::Phase::Assembly);

  if (this->is_output()) {
    validate(this->container() == reaction->container(),
             "Antidependent output ports must belong to the same reactor as "
             "the reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

  reactor_assert(anti_dependencies_.insert(reaction).second);
}

[[maybe_unused]] auto Port<void>::typed_outward_bindings() const noexcept -> const std::set<Port<void>*>& {
  // this is undefined behavior and should be changed
  return reinterpret_cast<const std::set<Port<void>*>&>(outward_bindings()); // NOLINT C++20 std::bit_cast
}

auto Port<void>::typed_inward_binding() const noexcept -> Port<void>* {
  // we can use a reinterpret cast here since we know that this port is always
  // connected with another Port<T>.
  return reinterpret_cast<Port<void>*>(inward_binding()); // NOLINT
}

void Port<void>::set() {
  validate(!has_inward_binding(), "set() may only be called on ports that do not have an inward "
                                  "binding!");

  auto* scheduler = environment()->scheduler();
  scheduler->set_port(this);
  this->present_ = true;
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
