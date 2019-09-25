/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

namespace dear {

template <class T>
const std::set<Port<T>*>& Port<T>::typed_outward_bindings() const {
  // HACK this cast is ugly but should be safe as long as we only allow to
  // bind with Input<T>*. The alternative would be to copy the entire set and
  // cast each element individually, which is also ugly...
  return reinterpret_cast<const std::set<Port<T>*>&>(outward_bindings());
}

template <class T>
Port<T>* Port<T>::typed_inward_binding() const {
  return dynamic_cast<Port<T>*>(inward_binding());
}

}  // namespace dear
