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

void BasePort::register_dependency(Reaction* reaction, bool is_trigger) noexcept {
  reactor_assert(reaction != nullptr);
  reactor_assert(this->environment() == reaction->environment()); // NOLINT
  // validate(!this->has_outward_bindings(), "Dependencies may no be declared on ports with an outward binding!");
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
  // validate(!this->has_inward_binding(), "Antidependencies may no be declared on ports with an inward binding!");
  assert_phase(this, Environment::Phase::Assembly);

  if (this->is_output()) {
    validate(this->container() == reaction->container(),
             "Antidependent output ports must belong to the same reactor as "
             "the reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Antidependent input ports must belong to a contained reactor");
  }
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
