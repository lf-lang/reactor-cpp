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

 protected:
  BasePort(const std::string& name, PortType type, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Port, container)
      , type(type) {}

  void base_bind_to(BasePort* port);

 public:
  bool is_input() const { return type == PortType::Input; }
  bool is_output() const { return type == PortType::Output; }

  bool has_inward_binding() const { return _inward_binding != nullptr; }
  bool has_outward_bindings() const { return _outward_bindings.size() > 0; }

  BasePort* inward_binding() const { return _inward_binding; }
  const auto& outward_bindings() const { return _outward_bindings; }
};

template <class T>
class Port : public BasePort {
 public:
  Port(const std::string& name, PortType type, Reactor* container)
      : BasePort(name, type, container) {}

  void bind_to(Port<T>* port) { base_bind_to(port); }
  Port<T>* typed_inward_binding() const;
  const std::set<Port<T>*>& typed_outward_bindings() const;
};

}  // namespace dear

#include "impl/port_impl.hh"
