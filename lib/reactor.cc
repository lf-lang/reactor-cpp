/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/reactor.hh"
#include "dear/action.hh"
#include "dear/environment.hh"
#include "dear/port.hh"
#include "dear/reaction.hh"
#include "dear/assert.hh"


namespace dear {

ReactorElement::ReactorElement(const std::string& name,
                               ReactorElement::Type type,
                               Reactor* container)
    : _name(name), _container(container) {
  ASSERT(container != nullptr);
  this->_environment = container->environment();
  ASSERT(this->_environment != nullptr);
  ASSERT(this->_environment->phase() == Environment::Phase::Construction);
  // We need a reinterpret_cast here as the derived class is not yet created
  // when this constructor is executed. dynamic_cast only works for
  // completely constructed objects. Technically, the casts here return
  // invalid pointers as the objects they point to do not yet
  // exists. However, we are good as long as we only store the pointer and do
  // not dereference it before construction is completeted.
  // It works, but maybe there is some nicer way of doing this...
  switch (type) {
    case Type::Action:
      container->register_action(reinterpret_cast<BaseAction*>(this));
      break;
    case Type::Port:
      container->register_port(reinterpret_cast<BasePort*>(this));
      break;
    case Type::Reaction:
      container->register_reaction(reinterpret_cast<Reaction*>(this));
      break;
    case Type::Reactor:
      container->register_reactor(reinterpret_cast<Reactor*>(this));
      break;
    default:
      ASSERT(false);
  }
}

ReactorElement::ReactorElement(const std::string& name,
                               ReactorElement::Type type,
                               Environment* environment)
    : _name(name), _container(nullptr), _environment(environment) {
  ASSERT(environment != nullptr);
  ASSERT(type == Type::Reactor);
  ASSERT(environment->phase() == Environment::Phase::Construction);
}

std::stringstream& ReactorElement::fqn_detail(std::stringstream& ss) const {
  if (this->is_top_level())
    ss << this->_name;
  else
    this->_container->fqn_detail(ss) << '.' << this->_name;
  return ss;
}

std::string ReactorElement::fqn() const {
  std::stringstream ss;
  return this->fqn_detail(ss).str();
}

Reactor::Reactor(const std::string& name, Reactor* container)
    : ReactorElement(name, ReactorElement::Type::Reactor, container) {}
Reactor::Reactor(const std::string& name, Environment* environment)
    : ReactorElement(name, ReactorElement::Type::Reactor, environment) {
  environment->register_reactor(this);
}

void Reactor::register_action(BaseAction* action) {
  ASSERT(action != nullptr);
  ASSERT(this->environment()->phase() == Environment::Phase::Construction);
  auto result = _actions.insert(action);
  ASSERT(result.second);
}
void Reactor::register_port(BasePort* port) {
  ASSERT(port != nullptr);
  ASSERT(this->environment()->phase() == Environment::Phase::Construction);
  if (port->is_input()) {
    auto result = _inputs.insert(port);
    ASSERT(result.second);
  } else {
    auto result = _outputs.insert(port);
    ASSERT(result.second);
  }
}
void Reactor::register_reaction(Reaction* reaction) {
  ASSERT(reaction != nullptr);
  ASSERT(this->environment()->phase() == Environment::Phase::Construction);
  auto result = _reactions.insert(reaction);
  ASSERT(result.second);
}
void Reactor::register_reactor(Reactor* reactor) {
  ASSERT(reactor != nullptr);
  ASSERT(this->environment()->phase() == Environment::Phase::Construction);
  auto result = _reactors.insert(reactor);
  ASSERT(result.second);
}

void Reactor::init(const Tag& t0) {
  ASSERT(environment()->phase() == Environment::Phase::Initialization);
  // call init on all contained objects
  for (auto x : _actions)
    x->init(t0);
  for (auto x : _inputs)
    x->init(t0);
  for (auto x : _outputs)
    x->init(t0);
  for (auto x : _reactions)
    x->init(t0);
  for (auto x : _reactors)
    x->init(t0);
}

}  // namespace dear
