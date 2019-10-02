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
 private:
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

  void init(const Tag&) override final {}
};

class Timer : public BaseAction {
  const time_t _offset;
  const time_t _period;

  void reschedule();

 public:
  Timer(const std::string& name,
        Reactor* container,
        time_t period = 0,
        time_t offset = 0)
      : BaseAction(name, container), _offset(offset), _period(period) {}

  void init(const Tag& t0) override final;

  time_t offset() const { return _offset; }
  time_t period() const { return _period; }
};

}  // namespace dear
