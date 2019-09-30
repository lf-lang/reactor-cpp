/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include "reactor.hh"

namespace dear {

class BaseAction : public ReactorElement {
  std::set<Reaction*> _triggers;
  std::set<Reaction*> _schedulers;

 protected:
  void register_trigger(Reaction* reaction);
  void register_scheduler(Reaction* reaction);

 public:
  BaseAction(const std::string& name, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Action, container) {}

  const auto& triggers() const { return _triggers; }
  const auto& schedulers() const { return _schedulers; }

  friend class Reaction;
};

template <class T>
class Action : public BaseAction {
 public:
  Action(const std::string& name, Reactor* container)
      : BaseAction(name, container) {}
};

}  // namespace dear
