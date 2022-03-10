/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include <utility>

#include "reactor-cpp/reaction.hh"

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/port.hh"

namespace reactor {

Reaction::Reaction(const std::string& name, int priority, Reactor* container, std::function<void(void)> body)
    : ReactorElement(name, ReactorElement::Type::Reaction, container)
    , priority_(priority)
    , body_(std::move(std::move(body))) {
  reactor_assert(priority != 0);
}

void Reaction::declare_trigger(BaseAction* action) {
  reactor_assert(action != nullptr);
  reactor_assert(this->environment() == action->environment());
  assert_phase(this, Environment::Phase::Assembly);
  validate(this->container() == action->container(), "Action triggers must belong to the same reactor as the triggered "
                                                     "reaction");

  reactor_assert(action_triggers_.insert(action).second);
  action->register_trigger(this);
}

void Reaction::declare_schedulable_action(BaseAction* action) {
  reactor_assert(action != nullptr);
  reactor_assert(this->environment() == action->environment());
  assert_phase(this, Environment::Phase::Assembly);
  validate(this->container() == action->container(), "Scheduable actions must belong to the same reactor as the "
                                                     "triggered reaction");

  reactor_assert(scheduable_actions_.insert(action).second);
  action->register_scheduler(this);
}

void Reaction::declare_trigger(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor_assert(this->environment() == port->environment());
  reactor_assert(this->environment()->phase() == Environment::Phase::Assembly);
  assert_phase(this, Environment::Phase::Assembly);

  if (port->is_input()) {
    validate(this->container() == port->container(),
             "Input port triggers must belong to the same reactor as the triggered "
             "reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Output port triggers must belong to a contained reactor");
  }

  reactor_assert(port_trigger_.insert(port).second);
  reactor_assert(dependencies_.insert(port).second);
  port->register_dependency(this, true);
}

void Reaction::declare_dependency(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor_assert(this->environment() == port->environment());
  assert_phase(this, Environment::Phase::Assembly);

  if (port->is_input()) {
    validate(this->container() == port->container(), "Dependent input ports must belong to the same reactor as the "
                                                     "reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Dependent output ports must belong to a contained reactor");
  }

  reactor_assert(dependencies_.insert(port).second);
  port->register_dependency(this, false);
}

void Reaction::declare_antidependency(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor_assert(this->environment() == port->environment());
  assert_phase(this, Environment::Phase::Assembly);

  if (port->is_output()) {
    validate(this->container() == port->container(), "Antidependent output ports must belong to the same reactor as "
                                                     "the reaction");
  } else {
    validate(this->container() == port->container()->container(),
             "Antidependent input ports must belong to a contained reactor");
  }

  reactor_assert(antidependencies_.insert(port).second);
  port->register_antidependency(this);
}

void Reaction::trigger() {
  if (has_deadline()) {
    reactor_assert(deadline_handler_ != nullptr);
    auto lag = Reactor::get_physical_time() - container()->get_logical_time();
    if (lag > deadline_) {
      deadline_handler_();
      return;
    }
  }

  body_();
}

void Reaction::set_deadline_impl(Duration deadline, const std::function<void(void)>& handler) {
  reactor_assert(!has_deadline());
  reactor_assert(handler != nullptr);
  this->deadline_ = deadline;
  this->deadline_handler_ = handler;
}

void Reaction::set_index(unsigned index) {
  validate(this->environment()->phase() == Environment::Phase::Assembly,
           "Reaction indexes may only be set during assembly phase!");
  this->index_ = index;
}

} // namespace reactor
