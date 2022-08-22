/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_ACTION_HH
#define REACTOR_CPP_ACTION_HH

#include "logical_time.hh"
#include "reactor.hh"
#include "value_ptr.hh"

#include <map>
#include <mutex>

namespace reactor {

class BaseAction : public ReactorElement {
private:
  std::set<Reaction*> triggers_{};
  std::set<Reaction*> schedulers_{};
  const Duration min_delay_{};
  const bool logical_{true};

protected:
  void register_trigger(Reaction* reaction);
  void register_scheduler(Reaction* reaction);

  virtual void setup() noexcept = 0;
  virtual void cleanup() noexcept = 0;

  BaseAction(const std::string& name, Reactor* container, bool logical, Duration min_delay)
      : ReactorElement(name, ReactorElement::Type::Action, container)
      , min_delay_(min_delay)
      , logical_(logical) {}

public:
  [[nodiscard]] auto inline triggers() const noexcept -> const auto& { return triggers_; }

  [[nodiscard]] auto inline schedulers() const noexcept -> const auto& { return schedulers_; }

  [[nodiscard]] auto inline is_logical() const noexcept -> bool { return logical_; }

  [[nodiscard]] auto inline is_physical() const noexcept -> bool { return !logical_; }

  [[nodiscard]] auto inline min_delay() const noexcept -> Duration { return min_delay_; }

  friend class Reaction;
  friend class Scheduler;
};

template <class T> class Action : public BaseAction {
private:
  ImmutableValuePtr<T> value_ptr_{nullptr};

  std::map<Tag, ImmutableValuePtr<T>> events_;
  std::mutex mutex_events_;

  void setup() noexcept final;
  void cleanup() noexcept final { value_ptr_ = nullptr; }

protected:
  Action(const std::string& name, Reactor* container, bool logical, Duration min_delay)
      : BaseAction(name, container, logical, min_delay) {}

public:
  void startup() final {}
  void shutdown() final {}

  template <class Dur = Duration> void schedule(const ImmutableValuePtr<T>& value_ptr, Dur delay = Dur::zero());

  template <class Dur = Duration> void schedule(MutableValuePtr<T>&& value_ptr, Dur delay = Dur::zero()) {
    schedule(ImmutableValuePtr<T>(std::forward<MutableValuePtr<T>>(value_ptr)), delay);
  }

  template <class Dur = Duration> void schedule(const T& value, Dur delay = Dur::zero()) {
    schedule(make_immutable_value<T>(value), delay);
  }

  template <class Dur = Duration> void schedule(T&& value, Dur delay = Dur::zero()) {
    schedule(make_immutable_value<T>(std::forward<T>(value)), delay);
  }

  // Scheduling an action with nullptr value is not permitted.
  template <class Dur = Duration> void schedule(std::nullptr_t, Dur) = delete;

  [[nodiscard]] auto get() const noexcept -> const ImmutableValuePtr<T>& { return value_ptr_; }

  [[nodiscard]] auto is_present() const noexcept -> bool { return value_ptr_ != nullptr; }
};

template <> class Action<void> : public BaseAction {
private:
  bool present_{false};

  void cleanup() noexcept final { present_ = false; }
  void setup() noexcept final { present_ = true; }

protected:
  Action(const std::string& name, Reactor* container, bool logical, Duration min_delay)
      : BaseAction(name, container, logical, min_delay) {}

public:
  void startup() final {}
  void shutdown() final {}

  template <class Dur = Duration> void schedule(Dur delay = Dur::zero());

  [[nodiscard]] auto is_present() const noexcept -> bool { return present_; }
};

template <class T> class PhysicalAction : public Action<T> {
public:
  PhysicalAction(const std::string& name, Reactor* container)
      : Action<T>(name, container, false, Duration::zero()) {}
};

template <class T> class LogicalAction : public Action<T> {
public:
  LogicalAction(const std::string& name, Reactor* container, Duration min_delay = Duration::zero())
      : Action<T>(name, container, true, min_delay) {}
};

class Timer : public BaseAction {
private:
  const Duration offset_{};
  const Duration period_{};

  void cleanup() noexcept final;

public:
  Timer(const std::string& name, Reactor* container, Duration period = Duration::zero(),
        Duration offset = Duration::zero())
      : BaseAction(name, container, true, Duration::zero())
      , offset_(offset)
      , period_(period) {}

  void startup() final;
  void shutdown() final {}
  void setup() noexcept final {}

  [[nodiscard]] auto offset() const noexcept -> const Duration& { return offset_; }

  [[nodiscard]] auto period() const noexcept -> const Duration& { return period_; }
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

  void setup() noexcept final {}
  void cleanup() noexcept final {}
  void startup() final {}
  void shutdown() final;
};

} // namespace reactor

#include "impl/action_impl.hh"

#endif // REACTOR_CPP_ACTION_HH
