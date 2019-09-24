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

class BaseOutput : public ReactorElement {
 private:
  std::set<BaseInput*> bindings;

 protected:
  void bind_base(BaseInput* output);

 public:
  BaseOutput(const std::string& name, Reactor* container)
      : ReactorElement(name, ReactorElement::Type::Output, container) {}

  bool is_bound() const { return bindings.size() > 0; }
  const auto& get_bindings() const { return bindings; }

  template <class T>
  friend class Input;
};

template <class T>
class Output : public BaseOutput {
 public:
  Output(const std::string& name, Reactor* container)
      : BaseOutput(name, container) {}

  void bind(Input<T>* input);
  const std::set<Input<T>*>& get_typed_bindings() const;
};

}  // namespace dear

#include "impl/output_impl.hh"
