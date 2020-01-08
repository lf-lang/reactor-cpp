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
#include "reactor-cpp/validate.hh"

namespace reactor {

void BasePort::base_bind_to(BasePort* port) {
  ASSERT(port != nullptr);
  ASSERT(this->environment() == port->environment());
  validate(!port->has_inward_binding(), "Ports may only be connected once");
  validate(!this->has_dependencies(),
           "Ports with dependencies may not be connected to other ports");
  validate(!this->has_antidependencies(),
           "Ports with antidependencies may not be connected to other ports");
  validate(this->environment()->phase() == Environment::Phase::Construction,
           "Ports can only be bound during contruction phase!");

  if (this->is_input() && port->is_input()) {
    validate(
        this->container() == port->container()->container(),
        "An input port A may only be bound to another input port B if B is "
        "contained by a reactor that in turn is contained by the reactor of A");
  } else if (this->is_output() && port->is_input()) {
    validate(this->container()->container() == port->container()->container(),
             "An output port can only be bound to an input port if both ports "
             "belong to reactors in the same hierarichal level");
    validate(this->container() != port->container(),
             "An output port can only be bound to an input port if both ports "
             "belong to different reactors!");
  } else if (this->is_output() && port->is_output()) {
    validate(
        this->container()->container() == port->container(),
        "An output port A may only be bound to another output port B if A is "
        "contained by a reactor that in turn is contained by the reactor of B");
  } else {
    throw std::runtime_error("unexpected case");
  }

  port->_inward_binding = this;
  auto result = this->_outward_bindings.insert(port);
  ASSERT(result.second);
}

void BasePort::register_dependency(Reaction* reaction, bool is_trigger) {
  ASSERT(reaction != nullptr);
  ASSERT(this->environment() == reaction->environment());
  validate(!this->has_outward_bindings(),
           "Dependencies may no be declared on ports with an outward binding!");
  validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Dependencies can only be registered during assembly phase!");

  if (this->is_input()) {
    validate(this->container() == reaction->container(),
             "Dependent input ports must belong to the same reactor as the "
             "reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Dependent output ports must belong to a contained reactor");
  }

  auto r1 = _dependencies.insert(reaction);
  ASSERT(r1.second);
  if (is_trigger) {
    auto r2 = _triggers.insert(reaction);
    ASSERT(r2.second);
  }
}

void BasePort::register_antidependency(Reaction* reaction) {
  ASSERT(reaction != nullptr);
  ASSERT(this->environment() == reaction->environment());
  validate(
      !this->has_inward_binding(),
      "Antidependencies may no be declared on ports with an inward binding!");
  validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Antidependencies can only be registered during assembly phase!");

  if (this->is_output()) {
    validate(this->container() == reaction->container(),
             "Antidependent output ports must belong to the same reactor as "
             "the reaction");
  } else {
    validate(this->container()->container() == reaction->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

  auto r = _antidependencies.insert(reaction);
  ASSERT(r.second);
}

const std::set<Port<void>*>& Port<void>::typed_outward_bindings() const {
  return reinterpret_cast<const std::set<Port<void>*>&>(outward_bindings());
}

Port<void>* Port<void>::typed_inward_binding() const {
  return dynamic_cast<Port<void>*>(inward_binding());
}

void Port<void>::set() {
  validate(!has_inward_binding(),
           "set() may only be called on a ports that do not have an inward "
           "binding!");
  auto scheduler = environment()->scheduler();
  this->present = true;
  scheduler->set_port(this);
}

}  // namespace reactor
