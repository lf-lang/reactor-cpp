/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include <cassert>

#include "dear/input.hh"

namespace dear {

template <class T>
void Output<T>::bind(Input<T>* input) {
  assert(input != nullptr);
  this->bind_base(input);
  input->bind_base(this);
}

template <class T>
const std::set<Input<T>*>& Output<T>::get_typed_bindings() const {
  // HACK this cast is ugly but should be safe as long as we only allow to
  // bind with Input<T>*. The alternative would be to copy the entire set and
  // cast each element individually, which is also ugly...
  return reinterpret_cast<const std::set<Input<T>*>&>(get_bindings());
}

}  // namespace dear
