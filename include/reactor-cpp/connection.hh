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

  [[nodiscard]] auto downstream_ports() -> auto& { return downstream_ports_; }
  [[nodiscard]] auto downstream_ports() const -> const auto& { return downstream_ports_; }

  void bind_upstream_port(Port<T>* port) {
    reactor_assert(upstream_port_ == nullptr);
    upstream_port_ = port;
    port->register_set_callback(upstream_set_callback());
  }

  void bind_downstream_port(Port<T>* port) { reactor_assert(this->downstream_ports_.insert(port).second); };

  virtual auto upstream_set_callback() noexcept -> PortCallback = 0;
};

template <class T> class DelayedConnection : Connection<T> {
public:
  DelayedConnection(const std::string& name, Reactor* container, Duration delay)
      : Connection<T>(name, container, true, delay) {}

  inline auto upstream_set_callback() noexcept -> PortCallback override {
    return [this](const BasePort& port) {
      // We know that port must be of type Port<T>*
      auto& typed_port = reinterpret_cast<const Port<T>&>(port); // NOLINT
      this->schedule(std::move(typed_port.get()));
      return true;
    };
  }

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

  using Connection<T>::bind_upstream_port;
  using Connection<T>::bind_downstream_port;
};

} // namespace reactor

#endif // REACTOR_CPP_CONNECTION_HH
