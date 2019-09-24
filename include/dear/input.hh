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
 private:
  BaseOutput* binding;

 protected:
  void bind_base(BaseOutput* output);

 public:
  BaseInput(const std::string& name, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Input, container)
      , binding(nullptr) {}

  bool is_bound() const { return binding != nullptr; }
  BaseOutput* get_binding() const { return binding; }

  template <class T>
  friend class Output;
};

template <class T>
class Input : public BaseInput {
 public:
  Input(const std::string& name, Reactor* container)
      : BaseInput(name, container) {}

  void bind(Output<T>* output);
  Output<T>* get_typed_binding() const;
};

}  // namespace dear

#include "impl/input_impl.hh"
