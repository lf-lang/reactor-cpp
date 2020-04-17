/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <functional>
#include <set>

#include "reactor.hh"

namespace reactor {

class Reaction : public ReactorElement {
 private:
  std::set<BaseAction*> _action_triggers;
  std::set<BaseAction*> _scheduable_actions;
  std::set<BasePort*> _port_triggers;
  std::set<BasePort*> _antidependencies;
  std::set<BasePort*> _dependencies;

  const int _priority;
  unsigned _index;

  std::function<void(void)> body;

  Duration deadline{Duration::zero()};
  std::function<void(void)> deadline_handler{nullptr};

  void set_deadline_impl(Duration deadline, std::function<void(void)> handler);

 public:
  Reaction(const std::string& name,
           int priority,
           Reactor* container,
           std::function<void(void)> body);

  virtual ~Reaction() {}

  void declare_trigger(BaseAction* action);
  void declare_trigger(BasePort* port);
  void declare_scheduable_action(BaseAction* action);
  void declare_antidependency(BasePort* port);
  void declare_dependency(BasePort* port);

  const auto& action_triggers() const { return _action_triggers; }
  const auto& port_triggers() const { return _port_triggers; }
  const auto& antidependencies() const { return _antidependencies; }
  const auto& dependencies() const { return _dependencies; }
  const auto& scheduable_actions() const { return _scheduable_actions; }

  int priority() const { return _priority; }

  void startup() override final {}
  void shutdown() override final {}

  void trigger();

  template <class Dur>
  void set_deadline(Dur dl, std::function<void(void)> handler) {
    set_deadline_impl(std::chrono::duration_cast<Duration>(dl), handler);
  }
  bool has_deadline() const { return deadline != Duration::zero(); }

  void set_index(unsigned index);
  unsigned index() const { return _index; }
};

}  // namespace reactor
