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

class BaseReaction : public ReactorElement {
 public:
  BaseReaction(const std::string& name, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Reaction, container) {}
};

}  // namespace dear
