/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/port.hh"

#include <cassert>

namespace dear {

void BasePort::base_bind_to(BasePort* port) {
  assert(port != nullptr);
  assert(!port->has_inward_binding());
  if (this->is_input() && port->is_input()) {
    // If both ports are inputs, the other port must be owned by an inner
    // reactor and this port must be owned by the same reactor that contains
    // the inner reactor.
    assert(this->container() == port->container()->container());
  } else if (this->is_output() && port->is_input()) {
    // If we connect an input to an output, the containing reactors must be in
    // the same hierarchy level.
    assert(this->container()->container() == port->container()->container());
    // But both ports must belong to different reactors
    assert(this->container() != port->container());
  } else if (this->is_output() && port->is_output()) {
    // If both ports are outputs, this port must be owned by an inner
    // reactor and the other port must be owned by the same reactor that
    // contains the inner reactor.
    assert(this->container()->container() == port->container());
  } else {
    assert(false);
  }

  port->_inward_binding = this;
  auto result = this->_outward_bindings.insert(port);
  assert(result.second);
}

}  // namespace dear
