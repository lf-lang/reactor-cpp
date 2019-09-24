/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include <cassert>

#include "dear/output.hh"

namespace dear {

template <class T>
void Input<T>::bind(Output<T>* output) {
  assert(output != nullptr);
  this->bind_base(output);
  output->bind_base(this);
}

template <class T>
Output<T>* Input<T>::get_typed_binding() const {
  return dynamic_cast<Output<T>*>(get_binding());
}

}  // namespace dear
