/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <set>
#include <sstream>
#include <string>

#include "fwd.hh"

namespace dear {

class ReactorElement {
 public:
  enum class Type { Reactor };

 private:
  const std::string _name;

  /// The reactor owning this element
  Reactor* const _container;

  std::stringstream& fqn_detail(std::stringstream& ss) const;

 public:
  ReactorElement(const std::string& name, Type type, Reactor* container);
  virtual ~ReactorElement() {}

  // not copyable or movable
  ReactorElement(const ReactorElement&) = delete;
  ReactorElement(ReactorElement&&) = delete;

  Reactor* container() const { return _container; }

  const std::string& name() const { return _name; }
  std::string fqn() const;

  bool is_top_level() const { return this->container() == nullptr; }
};

class Reactor : public ReactorElement {
 private:
  std::set<Reactor*> _reactors;

  void register_reactor(Reactor* reactor);

 public:
  Reactor(const std::string& name, Reactor* container = nullptr)
      : ReactorElement(name, ReactorElement::Type::Reactor, container) {}

  const auto& reactors() const { return _reactors; }

  friend ReactorElement;
};

}  // namespace dear
