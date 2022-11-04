/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <set>

#include "reactor.hh"
#include "value_ptr.hh"

namespace reactor {

enum class PortType { Input, Output };

class BasePort : public ReactorElement {
 private:
  BasePort* _inward_binding = nullptr;
  std::set<BasePort*> _outward_bindings;
  const PortType type;

  std::set<Reaction*> _dependencies;
  std::set<Reaction*> _triggers;
  std::set<Reaction*> _antidependencies;

 protected:
  BasePort(const std::string& name, PortType type, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Port, container)
      , type(type) {}

  void base_bind_to(BasePort* port);
  void register_dependency(Reaction* reaction, bool is_trigger);
  void register_antidependency(Reaction* reaction);

  virtual void cleanup() = 0;

 public:
  bool is_input() const { return type == PortType::Input; }
  bool is_output() const { return type == PortType::Output; }

  bool has_inward_binding() const { return _inward_binding != nullptr; }
  bool has_outward_bindings() const { return _outward_bindings.size() > 0; }
  bool has_dependencies() const { return _dependencies.size() > 0; }
  bool has_antidependencies() const { return _antidependencies.size() > 0; }

  BasePort* inward_binding() const { return _inward_binding; }
  const auto& outward_bindings() const { return _outward_bindings; }

  const auto& triggers() const { return _triggers; }
  const auto& dependencies() const { return _dependencies; }
  const auto& antidependencies() const { return _antidependencies; }

  friend class Reaction;
  friend class Scheduler;
};

template <class T>
class Port : public BasePort {
 private:
  ImmutableValuePtr<T> value_ptr{nullptr};

  void cleanup() override final { value_ptr = nullptr; }

 public:
  using value_type = T;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void bind_to(Port<T>* port) { base_bind_to(port); }
  Port<T>* typed_inward_binding() const;
  const std::set<Port<T>*>& typed_outward_bindings() const;

  void set(const ImmutableValuePtr<T>& value_ptr);
  void set(MutableValuePtr<T>&& value_ptr) {
    set(ImmutableValuePtr<T>(std::forward<MutableValuePtr<T>>(value_ptr)));
  }
  void set(const T& value) { set(make_immutable_value<T>(value)); }
  void set(T&& value) { set(make_immutable_value<T>(std::forward<T>(value))); }
  // Setting a port to nullptr is not permitted.
  void set(std::nullptr_t) = delete;

  void startup() override final {}
  void shutdown() override final {}

  const ImmutableValuePtr<T>& get() const;
  bool is_present() const;

};

template <>
class Port<void> : public BasePort {
 private:
  bool present{false};

  void cleanup() override final { present = false; }

 public:
  using value_type = void;

  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void bind_to(Port<void>* port) { base_bind_to(port); }
  Port<void>* typed_inward_binding() const;
  const std::set<Port<void>*>& typed_outward_bindings() const;

  void set();
  bool is_present() const;

  void startup() override final {}
  void shutdown() override final {}
};

template <class T>
class Input : public Port<T> {
 public:
  Input(const std::string& name, Reactor* container)
      : Port<T>(name, PortType::Input, container) {}
};

template <class T>
class Output : public Port<T> {
 public:
  Output(const std::string& name, Reactor* container)
      : Port<T>(name, PortType::Output, container) {}
};

}  // namespace reactor

#include "impl/port_impl.hh"
