/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <map>
#include <sstream>
#include <string>

namespace dear {

// forward declaration
class Reactor;

class ReactorElement {
 private:
  /// The reactor owning this element
  Reactor* const _container;

  const std::string _name;

  std::stringstream& fqn_detail(std::stringstream& ss) const;

 public:
  ReactorElement(const std::string& name, Reactor* container);
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
  std::map<std::string, ReactorElement*> elements;

  void insert_element(ReactorElement* elem);

 public:
  Reactor(const std::string& name, Reactor* container = nullptr)
      : ReactorElement(name, container) {}

  friend ReactorElement;
};

};  // namespace dear
