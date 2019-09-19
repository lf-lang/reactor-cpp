/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/reactor.hh"

#include <cassert>

namespace dear {

ReactorElement::ReactorElement(const std::string& name, Reactor* container)
    : _container(container), _name(name) {
  if (container != nullptr) {
    container->insert_element(this);
  }
}

std::stringstream& ReactorElement::fqn_detail(std::stringstream& ss) const {
  if (this->is_top_level())
    ss << this->_name;
  else
    this->_container->fqn_detail(ss) << '.' << this->_name;
  return ss;
}

std::string ReactorElement::fqn() const {
  std::stringstream ss;
  return this->fqn_detail(ss).str();
}

void Reactor::insert_element(ReactorElement* elem) {
  auto r = this->elements.insert({elem->name(), elem});
  assert(r.second && "Tried to insert an element that already exists");
}

}  // namespace dear
