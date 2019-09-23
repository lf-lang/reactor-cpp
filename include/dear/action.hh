/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include "reactor.hh"

namespace dear {

class BaseAction : public ReactorElement {
 public:
  BaseAction(const std::string& name, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Action, container) {}
};

template <class T>
class Action : public BaseAction {
  Action(const std::string& name, Reactor* container)
      : BaseAction(name, container) {}
};

}  // namespace dear
