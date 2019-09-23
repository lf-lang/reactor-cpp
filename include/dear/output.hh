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

class BaseOutput : public ReactorElement {
 public:
  BaseOutput(const std::string& name, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Output, container) {}
};

template <class T>
class Output : public BaseOutput {
  Output(const std::string& name, Reactor* container)
      : BaseOutput(name, container) {}
};

}  // namespace dear
