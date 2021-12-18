/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include <utility>

#include <utility>

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
    , body(std::move(std::move(body))) {
    reactor_assert(priority != 0);
}

void Reaction::declare_trigger(BaseAction* action) {
    reactor_assert(action != nullptr);
    reactor_assert(this->environment() == action->environment());
  validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Triggers may only be declared during assembly phase!");
  validate(this->container() == action->container(),
           "Action triggers must belong to the same reactor as the triggered "
           "reaction");

    reactor_assert(_action_triggers.insert(action).second);
  action->register_trigger(this);
}

void Reaction::declare_schedulable_action(BaseAction* action) {
    reactor_assert(action != nullptr);
    reactor_assert(this->environment() == action->environment());
  validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Scheduable actions may only be declared during assembly phase!");
  validate(this->container() == action->container(),
           "Scheduable actions must belong to the same reactor as the "
           "triggered reaction");

    reactor_assert(_scheduable_actions.insert(action).second);
  action->register_scheduler(this);
}

void Reaction::declare_trigger(BasePort* port) {
    reactor_assert(port != nullptr);
    reactor_assert(this->environment() == port->environment());
    reactor_assert(this->environment()->phase() == Environment::Phase::Assembly);
  validate(this->environment()->phase() == Environment::Phase::Assembly,
        "Triggers may only be declared during assembly phase!");

  if (port->is_input()) {
    validate(
        this->container() == port->container(),
        "Input port triggers must belong to the same reactor as the triggered "
        "reaction");
  } else {
    validate(this->container() == port->container()->container(),
        "Output port triggers must belong to a contained reactor");
  }

    reactor_assert(_port_triggers.insert(port).second);
    reactor_assert(_dependencies.insert(port).second);
  port->register_dependency(this, true);
}

void Reaction::declare_dependency(BasePort* port) {
    reactor_assert(port != nullptr);
    reactor_assert(this->environment() == port->environment());
  validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Dependencies may only be declared during assembly phase!");

  if (port->is_input()) {
    validate(this->container() == port->container(),
             "Dependent input ports must belong to the same reactor as the "
             "reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Dependent output ports must belong to a contained reactor");
  }

    reactor_assert(_dependencies.insert(port).second);
  port->register_dependency(this, false);
}

void Reaction::declare_antidependency(BasePort* port) {
    reactor_assert(port != nullptr);
    reactor_assert(this->environment() == port->environment());
  validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Antidependencies may only be declared during assembly phase!");

  if (port->is_output()) {
    validate(this->container() == port->container(),
             "Antidependent output ports must belong to the same reactor as "
             "the reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

    reactor_assert(_antidependencies.insert(port).second);
  port->register_antidependency(this);
}

void Reaction::trigger() {
  if (has_deadline()) {
      reactor_assert(deadline_handler != nullptr);
    auto lag =
        container()->get_physical_time() - container()->get_logical_time();
    if (lag > deadline) {
      deadline_handler();
      return;
    }
  }

  body();
}

void Reaction::set_deadline_impl(Duration dl,
                                 const std::function<void(void)>& handler) {
    reactor_assert(!has_deadline());
    reactor_assert(handler != nullptr);
  this->deadline = dl;
  this->deadline_handler = handler;
}

void Reaction::set_index(unsigned index) {
  validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Reaction indexes may only be set during assembly phase!");
  this->_index = index;
}

}  // namespace reactor
