/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/reaction.hh"
#include "dear/action.hh"
#include "dear/port.hh"

#include <cassert>

namespace dear {

Reaction::Reaction(const std::string& name, int priority, Reactor* container)
    : ReactorElement(name, ReactorElement::Type::Reaction, container)
    , _priority(priority) {
  assert(priority != 0);
}

void Reaction::declare_trigger(BaseAction* action) {
  assert(action != nullptr);
  // this reaction must belong to the same reactor as the action
  assert(this->container() == action->container());

  auto r = _action_triggers.insert(action);
  assert(r.second);
  action->register_trigger(this);
}

void Reaction::declare_scheduable_action(BaseAction* action) {
  assert(action != nullptr);
  // this reaction must belong to the same reactor as the action
  assert(this->container() == action->container());

  auto r = _scheduable_actions.insert(action);
  assert(r.second);
  action->register_scheduler(this);
}

void Reaction::declare_trigger(BasePort* port) {
  assert(port != nullptr);
  if (port->is_input()) {
    // this reaction must belong to the same reactor as the input port
    assert(this->container() == port->container());
  } else {
    // the reactor containing this reaction must contain the reactor that the
    // output port belongs to.
    assert(this->container() == port->container()->container());
  }

  auto r1 = _port_triggers.insert(port);
  assert(r1.second);
  auto r2 = _dependencies.insert(port);
  assert(r2.second);
  port->register_dependency(this, true);
}

void Reaction::declare_dependency(BasePort* port) {
  assert(port != nullptr);
  if (port->is_input()) {
    // this reaction must belong to the same reactor as the input port
    assert(this->container() == port->container());
  } else {
    // the reactor containing this reaction must contain the reactor that the
    // input port belongs to.
    assert(this->container() == port->container()->container());
  }

  auto r = _dependencies.insert(port);
  assert(r.second);
  port->register_dependency(this, false);
}

void Reaction::declare_antidependency(BasePort* port) {
  assert(port != nullptr);
  if (port->is_output()) {
    // this reaction must belong to the same reactor as the output port
    assert(this->container() == port->container());
  } else {
    // the reactor containing this reaction must contain the reactor that the
    // output port belongs to.
    assert(this->container() == port->container()->container());
  }

  auto r = _antidependencies.insert(port);
  assert(r.second);
  port->register_antidependency(this);
}

}  // namespace dear
