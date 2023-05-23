/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/reactor.hh"

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/statistics.hh"

namespace reactor {

ReactorElement::ReactorElement(const std::string& name, ReactorElement::Type type, Reactor* container)
    : name_(name)
    , container_(container) {
  reactor_assert(container != nullptr);
  this->environment_ = container->environment(); // NOLINT container can be NULL
  reactor_assert(this->environment_ != nullptr);
  validate(this->environment_->phase() == Phase::Construction ||
               (type == Type::Action && this->environment_->phase() == Phase::Assembly),
           "Reactor elements can only be created during construction phase!");
  // We need a reinterpret_cast here as the derived class is not yet created
  // when this constructor is executed. dynamic_cast only works for
  // completely constructed objects. Technically, the casts here return
  // invalid pointers as the objects they point to do not yet
  // exists. However, we are good as long as we only store the pointer and do
  // not dereference it before construction is completeted.
  // It works, but maybe there is some nicer way of doing this...
  switch (type) {
  case Type::Action:
    container->register_action(reinterpret_cast<BaseAction*>(this)); // NOLINT
    break;
  case Type::Input:
    container->register_input(reinterpret_cast<BasePort*>(this)); // NOLINT
    break;
  case Type::Output:
    container->register_output(reinterpret_cast<BasePort*>(this)); // NOLINT
    break;
  case Type::Reaction:
    container->register_reaction(reinterpret_cast<Reaction*>(this)); // NOLINT
    break;
  case Type::Reactor:
    container->register_reactor(reinterpret_cast<Reactor*>(this)); // NOLINT
    break;
  default:
    throw std::runtime_error("unexpected type");
  }

  std::stringstream string_stream;
  string_stream << container_->fqn() << '.' << name;
  fqn_ = string_stream.str();
}

ReactorElement::ReactorElement(const std::string& name, ReactorElement::Type type, Environment* environment)
    : name_(name)
    , fqn_(name)
    , environment_(environment) {
  reactor_assert(environment != nullptr);
  validate(type == Type::Reactor || type == Type::Action, "Only reactors and actions can be owned by the environment!");
  validate(this->environment_->phase() == Phase::Construction,
           "Reactor elements can only be created during construction phase!");

  switch (type) {
  case Type::Action:
    Statistics::increment_actions();
    break;
  case Type::Reactor:
    Statistics::increment_reactor_instances();
    break;
  default:
    break;
  }
}

Reactor::Reactor(const std::string& name, Reactor* container)
    : ReactorElement(name, ReactorElement::Type::Reactor, container) {}
Reactor::Reactor(const std::string& name, Environment* environment)
    : ReactorElement(name, ReactorElement::Type::Reactor, environment) {
  environment->register_reactor(this);
}

void Reactor::register_action([[maybe_unused]] BaseAction* action) {
  reactor_assert(action != nullptr);
  reactor::validate(this->environment()->phase() == Phase::Construction ||
                        this->environment()->phase() == Phase::Assembly,
                    "Actions can only be registered during construction phase!");
  [[maybe_unused]] bool result = actions_.insert(action).second;
  reactor_assert(result);
  Statistics::increment_actions();
}

void Reactor::register_input(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor::validate(this->environment()->phase() == Phase::Construction,
                    "Ports can only be registered during construction phase!");
  [[maybe_unused]] bool result = inputs_.insert(port).second;
  reactor_assert(result);
  Statistics::increment_ports();
}

void Reactor::register_output(BasePort* port) {
  reactor_assert(port != nullptr);
  reactor::validate(this->environment()->phase() == Phase::Construction,
                    "Ports can only be registered during construction phase!");
  [[maybe_unused]] bool result = inputs_.insert(port).second;
  reactor_assert(result);
  Statistics::increment_ports();
}

void Reactor::register_reaction([[maybe_unused]] Reaction* reaction) {
  reactor_assert(reaction != nullptr);

  validate(this->environment()->phase() == Phase::Construction,
           "Reactions can only be registered during construction phase!");
  [[maybe_unused]] bool result = reactions_.insert(reaction).second;
  reactor_assert(result);
  Statistics::increment_reactions();
}

void Reactor::register_reactor([[maybe_unused]] Reactor* reactor) {
  reactor_assert(reactor != nullptr);
  validate(this->environment()->phase() == Phase::Construction,
           "Reactions can only be registered during construction phase!");
  [[maybe_unused]] bool result = reactors_.insert(reactor).second;
  reactor_assert(result);
  Statistics::increment_reactor_instances();
}

void Reactor::startup() {
  reactor_assert(environment()->phase() == Phase::Startup);
  log::Debug() << "Starting up reactor " << fqn();
  // call startup on all contained objects
  for (auto* base_action : actions_) {
    base_action->startup();
  }
  for (auto* base_port : inputs_) {
    base_port->startup();
  }
  for (auto* base_port : outputs_) {
    base_port->startup();
  }
  for (auto* reaction : reactions_) {
    reaction->startup();
  }
  for (auto* reactor : reactors_) {
    reactor->startup();
  }
}

void Reactor::shutdown() {
  reactor_assert(environment()->phase() == Phase::Shutdown);
  log::Debug() << "Terminating reactor " << fqn();
  // call shutdown on all contained objects
  for (auto* action : actions_) {
    action->shutdown();
  }
  for (auto* base_port : inputs_) {
    base_port->shutdown();
  }
  for (auto* base_port : outputs_) {
    base_port->shutdown();
  }
  for (auto* reaction : reactions_) {
    reaction->shutdown();
  }
  for (auto* reactor : reactors_) {
    reactor->shutdown();
  }
}

auto Reactor::get_physical_time() noexcept -> TimePoint { return reactor::get_physical_time(); }

auto Reactor::get_logical_time() const noexcept -> TimePoint {
  return environment()->scheduler()->logical_time().time_point();
}

auto Reactor::get_microstep() const noexcept -> mstep_t {
  return environment()->scheduler()->logical_time().micro_step();
}

auto Reactor::get_tag() const noexcept -> Tag {
  return Tag::from_logical_time(environment()->scheduler()->logical_time());
}

auto Reactor::get_elapsed_logical_time() const noexcept -> Duration {
  return get_logical_time() - environment()->start_tag().time_point();
}

auto Reactor::get_elapsed_physical_time() const noexcept -> Duration {
  return get_physical_time() - environment()->start_tag().time_point();
}

} // namespace reactor
