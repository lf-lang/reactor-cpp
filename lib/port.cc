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
  toggle_assert(port != nullptr);
  toggle_assert(this->environment() == port->environment());
  reactor::validate(!port->has_inward_binding(), "Ports may only be connected once");
  reactor::validate(!this->has_dependencies(),
           "Ports with dependencies may not be connected to other ports");
  reactor::validate(!port->has_antidependencies(),
           "Ports with antidependencies may not be connected to other ports");
  reactor::validate(this->environment()->phase() == Environment::Phase::Construction,
           "Ports can only be bound during contruction phase!");

  if (this->is_input() && port->is_input()) {
    reactor::validate(
        this->container() == port->container()->container(),
        "An input port A may only be bound to another input port B if B is "
        "contained by a reactor that in turn is contained by the reactor of A");
  } else if (this->is_output() && port->is_input()) {
    reactor::validate(this->container()->container() == port->container()->container(),
             "An output port can only be bound to an input port if both ports "
             "belong to reactors in the same hierarichal level");
    reactor::validate(this->container() != port->container(),
             "An output port can only be bound to an input port if both ports "
             "belong to different reactors!");
  } else if (this->is_output() && port->is_output()) {
    reactor::validate(
        this->container()->container() == port->container(),
        "An output port A may only be bound to another output port B if A is "
        "contained by a reactor that in turn is contained by the reactor of B");
  } else {
    throw std::runtime_error("unexpected case");
  }

  port->_inward_binding = this;
  toggle_assert(this->_outward_bindings.insert(port).second);
}

void BasePort::register_dependency(Reaction* reaction, bool is_trigger) {
  toggle_assert(reaction != nullptr);
  toggle_assert(this->environment() == reaction->environment());
  reactor::validate(!this->has_outward_bindings(),
           "Dependencies may no be declared on ports with an outward binding!");
  reactor::validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Dependencies can only be registered during assembly phase!");

  if (this->is_input()) {
    reactor::validate(this->container() == reaction->container(),
             "Dependent input ports must belong to the same reactor as the "
             "reaction");
  } else {
    reactor::validate(this->container()->container() == reaction->container(),
             "Dependent output ports must belong to a contained reactor");
  }

  toggle_assert(_dependencies.insert(reaction).second);
  if (is_trigger) {
    toggle_assert(_triggers.insert(reaction).second);
  }
}

void BasePort::register_antidependency(Reaction* reaction) {
  toggle_assert(reaction != nullptr);
  toggle_assert(this->environment() == reaction->environment());
  reactor::validate(
      !this->has_inward_binding(),
      "Antidependencies may no be declared on ports with an inward binding!");
  reactor::validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Antidependencies can only be registered during assembly phase!");

  if (this->is_output()) {
    reactor::validate(this->container() == reaction->container(),
             "Antidependent output ports must belong to the same reactor as "
             "the reaction");
  } else {
    reactor::validate(this->container()->container() == reaction->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

  toggle_assert(_antidependencies.insert(reaction).second);
}

const std::set<Port<void>*>& Port<void>::typed_outward_bindings() const {
  return reinterpret_cast<const std::set<Port<void>*>&>(outward_bindings());
}

Port<void>* Port<void>::typed_inward_binding() const {
  // we can use a static cast here since we know that this port is always
  // connected with another Port<T>.
  return static_cast<Port<void>*>(inward_binding());
}

void Port<void>::set() {
  reactor::validate(!has_inward_binding(),
           "set() may only be called on a ports that do not have an inward "
           "binding!");
  auto scheduler = environment()->scheduler();
  this->present = true;
  scheduler->set_port(this);
}

bool Port<void>::is_present() const {
  if (has_inward_binding()) {
    return typed_inward_binding()->is_present();
  } else {
    return present;
  }
}

}  // namespace reactor
