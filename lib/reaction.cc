/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/reaction.hh"

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/port.hh"

namespace reactor {

Reaction::Reaction(const std::string& name,
                   int priority,
                   Reactor* container,
                   std::function<void(void)> body)
    : ReactorElement(name, ReactorElement::Type::Reaction, container)
    , _priority(priority)
    , body(body) {
  ASSERT(priority != 0);
}

void Reaction::declare_trigger(BaseAction* action) {
  ASSERT(action != nullptr);
  ASSERT(this->environment() == action->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);
  // this reaction must belong to the same reactor as the action
  ASSERT(this->container() == action->container());

  auto r = _action_triggers.insert(action);
  ASSERT(r.second);
  action->register_trigger(this);
}

void Reaction::declare_scheduable_action(BaseAction* action) {
  ASSERT(action != nullptr);
  ASSERT(this->environment() == action->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);
  // this reaction must belong to the same reactor as the action
  ASSERT(this->container() == action->container());

  auto r = _scheduable_actions.insert(action);
  ASSERT(r.second);
  action->register_scheduler(this);
}

void Reaction::declare_trigger(BasePort* port) {
  ASSERT(port != nullptr);
  ASSERT(this->environment() == port->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);

  if (port->is_input()) {
    // this reaction must belong to the same reactor as the input port
    ASSERT(this->container() == port->container());
  } else {
    // the reactor containing this reaction must contain the reactor that
    // the output port belongs to.
    ASSERT(this->container() == port->container()->container());
  }

  auto r1 = _port_triggers.insert(port);
  ASSERT(r1.second);
  auto r2 = _dependencies.insert(port);
  ASSERT(r2.second);
  port->register_dependency(this, true);
}

void Reaction::declare_dependency(BasePort* port) {
  ASSERT(port != nullptr);
  ASSERT(this->environment() == port->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);

  if (port->is_input()) {
    // this reaction must belong to the same reactor as the input port
    ASSERT(this->container() == port->container());
  } else {
    // the reactor containing this reaction must contain the reactor that
    // the input port belongs to.
    ASSERT(this->container() == port->container()->container());
  }

  auto r = _dependencies.insert(port);
  ASSERT(r.second);
  port->register_dependency(this, false);
}

void Reaction::declare_antidependency(BasePort* port) {
  ASSERT(port != nullptr);
  ASSERT(this->environment() == port->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);

  if (port->is_output()) {
    // this reaction must belong to the same reactor as the output port
    ASSERT(this->container() == port->container());
  } else {
    // the reactor containing this reaction must contain the reactor that
    // the output port belongs to.
    ASSERT(this->container() == port->container()->container());
  }

  auto r = _antidependencies.insert(port);
  ASSERT(r.second);
  port->register_antidependency(this);
}

void Reaction::trigger() {
  if (has_deadline()) {
    ASSERT(deadline_handler != nullptr);
    auto lag =
        container()->get_physical_time() - container()->get_logical_time();
    if (lag > deadline) {
      deadline_handler();
      return;
    }
  }

  body();
}

void Reaction::set_deadline(time_t dl,
                            std::function<void(void)> handler) {
  ASSERT(!has_deadline());
  ASSERT(handler != nullptr);
  this->deadline = dl;
  this->deadline_handler = handler;
}

}  // namespace reactor
