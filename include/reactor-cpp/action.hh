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

namespace reactor {

class BaseAction : public ReactorElement {
 private:
  std::set<Reaction*> _triggers;
  std::set<Reaction*> _schedulers;

  const bool _logical;

 protected:
  void register_trigger(Reaction* reaction);
  void register_scheduler(Reaction* reaction);

  virtual void cleanup() = 0;

  const Duration min_delay;

 protected:
  BaseAction(const std::string& name,
             Reactor* container,
             bool logical,
             Duration min_delay)
      : ReactorElement(name, ReactorElement::Type::Action, container)
      , _logical(logical)
      , min_delay(min_delay) {}

 public:
  const auto& triggers() const { return _triggers; }
  const auto& schedulers() const { return _schedulers; }

  bool is_logical() const { return _logical; }
  bool is_physical() const { return !_logical; }

  friend class Reaction;
  friend class Scheduler;
};

template <class T>
class Action : public BaseAction {
 private:
  ImmutableValuePtr<T> value_ptr{nullptr};

  void cleanup() override final { value_ptr = nullptr; }

 protected:
  Action(const std::string& name,
         Reactor* container,
         bool logical,
         Duration min_delay)
      : BaseAction(name, container, logical, min_delay) {}

 public:
  void startup() override final {}
  void shutdown() override final {}

  template <class Dur = Duration>
  void schedule(const ImmutableValuePtr<T>& value_ptr, Dur delay = Dur::zero());
  template <class Dur = Duration>
  void schedule(MutableValuePtr<T>&& value_ptr, Dur delay = Dur::zero()) {
    schedule(ImmutableValuePtr<T>(std::forward<MutableValuePtr<T>>(value_ptr)),
             delay);
  }
  template <class Dur = Duration>
  void schedule(const T& value, Dur delay = Dur::zero()) {
    schedule(make_immutable_value<T>(value), delay);
  }

  const ImmutableValuePtr<T>& get() const { return value_ptr; }
  bool is_present() const { return value_ptr != nullptr; }
};

template <>
class Action<void> : public BaseAction {
 private:
  bool present{false};

  void cleanup() override final { present = false; }

 protected:
  Action(const std::string& name,
         Reactor* container,
         bool logical,
         Duration min_delay)
      : BaseAction(name, container, logical, min_delay) {}

 public:
  void startup() override final {}
  void shutdown() override final {}

  template <class Dur = Duration>
  void schedule(Dur delay = Dur::zero());
  bool is_present() const { return present; }
};

template <class T>
class PhysicalAction : public Action<T> {
 public:
  PhysicalAction(const std::string& name, Reactor* container)
      : Action<T>(name, container, false, Duration::zero()) {}
};

template <class T>
class LogicalAction : public Action<T> {
 public:
  LogicalAction(const std::string& name,
                Reactor* container,
                Duration min_delay = Duration::zero())
      : Action<T>(name, container, true, min_delay) {}
};

class Timer : public BaseAction {
 private:
  const Duration _offset;
  const Duration _period;

  void reschedule();

  void cleanup() override final;

 public:
  Timer(const std::string& name,
        Reactor* container,
        Duration period = Duration::zero(),
        Duration offset = Duration::zero())
      : BaseAction(name, container, true, Duration::zero())
      , _offset(offset)
      , _period(period) {}

  void startup() override final;
  void shutdown() override final {}

  const Duration& offset() const { return _offset; }
  const Duration& period() const { return _period; }
};

class StartupAction : public Timer {
 public:
  StartupAction(const std::string& name, Reactor* container)
      : Timer(name, container) {}
};

class ShutdownAction : public BaseAction {
 public:
  ShutdownAction(const std::string& name, Reactor* container)
      : BaseAction(name, container, true, Duration::zero()) {}

  void cleanup() override final {}
  void startup() override final {}
  void shutdown() override final;
};

}  // namespace reactor

#include "impl/action_impl.hh"
