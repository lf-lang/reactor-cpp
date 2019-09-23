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

class BaseInput : public ReactorElement {
 public:
  BaseInput(const std::string& name, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Input, container) {}
};

template <class T>
class Input : public BaseInput {
  Input(const std::string& name, Reactor* container)
      : BaseInput(name, container) {}
};

}  // namespace dear
