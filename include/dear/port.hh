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

namespace dear {

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
};

template <class T>
class Port : public BasePort {
 public:
  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void bind_to(Port<T>* port) { base_bind_to(port); }
  Port<T>* typed_inward_binding() const;
  const std::set<Port<T>*>& typed_outward_bindings() const;

  void init(const Tag&) override final {}
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

}  // namespace dear

#include "impl/port_impl.hh"
