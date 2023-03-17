/*
 * Copyright (C) 2023 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 *   Christian Menard
 */

#ifndef REACTOR_CPP_CONNECTION_HH
#define REACTOR_CPP_CONNECTION_HH

#include "action.hh"
#include "assert.hh"
#include "environment.hh"
#include "fwd.hh"
#include "logical_time.hh"
#include "port.hh"
#include "reaction.hh"
#include "reactor.hh"
#include "time.hh"

namespace reactor {

template <class T> class Connection : public Action<T> {
private:
  Port<T>* upstream_port_{nullptr};
  std::set<Port<T>*> downstream_ports_{};

protected:
  Connection(const std::string& name, Reactor* container, bool is_logical, Duration min_delay)
      : Action<T>(name, container, is_logical, min_delay) {}
  Connection(const std::string& name, Environment* environment, bool is_logical, Duration min_delay)
      : Action<T>(name, environment, is_logical, min_delay) {}

  [[nodiscard]] auto downstream_ports() -> auto& { return downstream_ports_; }
  [[nodiscard]] auto downstream_ports() const -> const auto& { return downstream_ports_; }

  virtual auto upstream_set_callback() noexcept -> PortCallback = 0;

public:
  virtual void bind_upstream_port(Port<T>* port) {
    reactor_assert(upstream_port_ == nullptr);
    upstream_port_ = port;
    port->register_set_callback(upstream_set_callback());
  }

  virtual void bind_downstream_port(Port<T>* port) { reactor_assert(this->downstream_ports_.insert(port).second); };
};

template <class T> class BaseDelayedConnection : public Connection<T> {
protected:
  BaseDelayedConnection(const std::string& name, Reactor* container, bool is_logical, Duration delay)
      : Connection<T>(name, container, is_logical, delay) {}
  BaseDelayedConnection(const std::string& name, Environment* environment, bool is_logical, Duration delay)
      : Connection<T>(name, environment, is_logical, delay) {}

  inline auto upstream_set_callback() noexcept -> PortCallback override {
    return [this](const BasePort& port) {
      // We know that port must be of type Port<T>
      auto& typed_port = reinterpret_cast<const Port<T>&>(port); // NOLINT
      if constexpr (std::is_same<T, void>::value) {
        this->schedule();
      } else {
        this->schedule(std::move(typed_port.get()));
      }
    };
  }

public:
  void setup() noexcept override {
    Action<T>::setup();

    if constexpr (std::is_same<T, void>::value) {
      for (auto port : this->downstream_ports()) {
        port->set();
      }
    } else {
      for (auto port : this->downstream_ports()) {
        port->set(std::move(this->get()));
      }
    }
  }
};

template <class T> class DelayedConnection : public BaseDelayedConnection<T> {
public:
  DelayedConnection(const std::string& name, Reactor* container, Duration delay)
      : BaseDelayedConnection<T>(name, container, true, delay) {}
};

template <class T> class PhysicalConnection : public BaseDelayedConnection<T> {
public:
  PhysicalConnection(const std::string& name, Reactor* container, Duration delay)
      : BaseDelayedConnection<T>(name, container, false, delay) {}
};

template <class T> class EnclaveConnection : public BaseDelayedConnection<T> {
private:
  std::mutex mutex_{};
  LogicalTime released_time_;

  void release_tag(const LogicalTime& tag) {
    {
      std::lock_guard<std::mutex> lock_guard(mutex_);
      released_time_.advance_to(tag);
    }
    this->environment()->scheduler()->notify();
  }

public:
  EnclaveConnection(const std::string& name, Environment* enclave)
      : BaseDelayedConnection<T>(name, enclave, false, Duration::zero()) {}

  inline auto upstream_set_callback() noexcept -> PortCallback override {
    return [this](const BasePort& port) {
      // We know that port must be of type Port<T>
      auto& typed_port = reinterpret_cast<const Port<T>&>(port); // NOLINT
      const auto* scheduler = port.environment()->scheduler();
      // This callback will be called from a reaction executing in the context
      // of the downstream port. Hence, we can retrieve the current tag directly
      // without locking.
      auto tag = Tag::from_logical_time(scheduler->logical_time());
      bool result{false};
      if constexpr (std::is_same<T, void>::value) {
        result = this->schedule_at(tag);
      } else {
        result = this->schedule_at(std::move(typed_port.get()), tag);
      }
      reactor_assert(result);
    };
  }

  auto acquire_tag([[maybe_unused]] std::condition_variable& cv, [[maybe_unused]] const Tag& tag) -> bool override {
    std::unique_lock<std::mutex> lock_guard(mutex_);
    if (released_time_ >= tag) {
      return true;
    }
    cv.wait(lock_guard);
    return released_time_ >= tag;
  }

  void bind_upstream_port(Port<T>* port) override {
    Connection<T>::bind_upstream_port(port);
    port->environment()->scheduler()->register_release_tag_callback(
        [this](const LogicalTime& tag) { release_tag(tag); });
  };
};

} // namespace reactor

#endif // REACTOR_CPP_CONNECTION_HH
