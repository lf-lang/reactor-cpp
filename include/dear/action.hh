/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include "logical_time.hh"
#include "reactor.hh"
#include "value_ptr.hh"

namespace dear {

class BaseAction : public ReactorElement {
 private:
  std::set<Reaction*> _triggers;
  std::set<Reaction*> _schedulers;

 protected:
  void register_trigger(Reaction* reaction);
  void register_scheduler(Reaction* reaction);

  virtual void cleanup() = 0;

 public:
  BaseAction(const std::string& name, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Action, container) {}

  const auto& triggers() const { return _triggers; }
  const auto& schedulers() const { return _schedulers; }

  friend class Reaction;
  friend class Scheduler;
};

template <class T>
class Action : public BaseAction {
 private:
  ImmutableValuePtr<T> value_ptr{nullptr};

  void cleanup() override final { value_ptr = nullptr; }

 public:
  Action(const std::string& name, Reactor* container)
      : BaseAction(name, container) {}

  void init(const Tag&) override final {}

  void schedule(const ImmutableValuePtr<T>& value_ptr, time_t delay = 0);
  void schedule(MutableValuePtr<T>&& value_ptr, time_t delay = 0) {
    schedule(ImmutableValuePtr<T>(value_ptr), delay);
  }
  void schedule(const T& value, time_t delay = 0) {
    schedule(make_immutable_value<T>(value), delay);
  }

  const ImmutableValuePtr<T> get() const { return value_ptr; }
  bool is_present() const { return value_ptr != nullptr; }
};

template <>
class Action<void> : public BaseAction {
 private:
  bool present{false};

  void cleanup() override final { present = false; }

 public:
  Action(const std::string& name, Reactor* container)
      : BaseAction(name, container) {}

  void init(const Tag&) override final {}

  void schedule(time_t delay = 0);
  bool is_present() const { return present; }
};

class Timer : public BaseAction {
 private:
  const time_t _offset;
  const time_t _period;

  void reschedule();

  void cleanup() override final {}

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

#include "impl/action_impl.hh"
