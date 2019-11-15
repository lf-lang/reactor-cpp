/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <set>
#include <sstream>
#include <string>

#include "fwd.hh"
#include "time.hh"

namespace reactor {

class ReactorElement {
 public:
  enum class Type { Action, Port, Reaction, Reactor };

 private:
  const std::string _name;

  /// The reactor owning this element
  Reactor* const _container;

  Environment* _environment;

  std::stringstream& fqn_detail(std::stringstream& ss) const;

 public:
  ReactorElement(const std::string& name, Type type, Reactor* container);
  ReactorElement(const std::string& name, Type type, Environment* environment);
  virtual ~ReactorElement() {}

  // not copyable or movable
  ReactorElement(const ReactorElement&) = delete;
  ReactorElement(ReactorElement&&) = delete;

  Reactor* container() const { return _container; }

  const std::string& name() const { return _name; }
  Environment* environment() const { return _environment; }
  std::string fqn() const;

  bool is_top_level() const { return this->container() == nullptr; }

  virtual void startup() = 0;
};

class Reactor : public ReactorElement {
 private:
  std::set<BaseAction*> _actions;
  std::set<BasePort*> _inputs;
  std::set<BasePort*> _outputs;
  std::set<Reaction*> _reactions;
  std::set<Reactor*> _reactors;

  void register_action(BaseAction* action);
  void register_port(BasePort* port);
  void register_reaction(Reaction* reaction);
  void register_reactor(Reactor* reactor);

 public:
  Reactor(const std::string& name, Reactor* container);
  Reactor(const std::string& name, Environment* environment);
  virtual ~Reactor() {}

  const auto& actions() const { return _actions; }
  const auto& inputs() const { return _inputs; }
  const auto& outputs() const { return _outputs; }
  const auto& reactions() const { return _reactions; }
  const auto& reactors() const { return _reactors; }

  void startup() override final;

  virtual void assemble() = 0;

  time_t get_physical_time() const;
  time_t get_logical_time() const;
  time_t get_elapsed_logical_time() const;

  friend ReactorElement;
};

}  // namespace reactor
