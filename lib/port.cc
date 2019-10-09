/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/port.hh"
#include "dear/assert.hh"
#include "dear/environment.hh"
#include "dear/reaction.hh"

namespace dear {

void BasePort::base_bind_to(BasePort* port) {
  ASSERT(port != nullptr);
  ASSERT(!port->has_inward_binding());
  ASSERT(!this->has_dependencies());
  ASSERT(!port->has_antidependencies());
  ASSERT(this->environment() == port->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);

  if (this->is_input() && port->is_input()) {
    // If both ports are inputs, the other port must be owned by an inner
    // reactor and this port must be owned by the same reactor that contains
    // the inner reactor.
    ASSERT(this->container() == port->container()->container());
  } else if (this->is_output() && port->is_input()) {
    // If we connect an input to an output, the containing reactors must be in
    // the same hierarchy level.
    ASSERT(this->container()->container() == port->container()->container());
    // But both ports must belong to different reactors
    ASSERT(this->container() != port->container());
  } else if (this->is_output() && port->is_output()) {
    // If both ports are outputs, this port must be owned by an inner
    // reactor and the other port must be owned by the same reactor that
    // contains the inner reactor.
    ASSERT(this->container()->container() == port->container());
  } else {
    ASSERT(false);
  }

  port->_inward_binding = this;
  auto result = this->_outward_bindings.insert(port);
  ASSERT(result.second);
}

void BasePort::register_dependency(Reaction* reaction, bool is_trigger) {
  ASSERT(reaction != nullptr);
  ASSERT(!this->has_outward_bindings());
  ASSERT(this->environment() == reaction->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);

  if (this->is_input()) {
    // the reaction must belong to the same reactor as this input port
    ASSERT(this->container() == reaction->container());
  } else {
    // the reactor containing reaction must contain the reactor that this
    // input port belongs to.
    ASSERT(this->container()->container() == reaction->container());
  }

  auto r1 = _dependencies.insert(reaction);
  ASSERT(r1.second);
  if (is_trigger) {
    auto r2 = _triggers.insert(reaction);
    ASSERT(r2.second);
  }
}

void BasePort::register_antidependency(Reaction* reaction) {
  ASSERT(reaction != nullptr);
  ASSERT(!this->has_inward_binding());
  ASSERT(this->environment() == reaction->environment());
  ASSERT(this->environment()->phase() == Environment::Phase::Assembly);

  if (this->is_output()) {
    // the reaction must belong to the same reactor as this output port
    ASSERT(this->container() == reaction->container());
  } else {
    // the reactor containing reaction must contain the reactor that this
    // input port belongs to.
    ASSERT(this->container()->container() == reaction->container());
  }

  auto r = _antidependencies.insert(reaction);
  ASSERT(r.second);
}

}  // namespace dear
