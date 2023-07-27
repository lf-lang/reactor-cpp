/*
 * Copyright (C) 2023 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_CONNECTION_PROPERTIES_HH
#define REACTOR_CPP_CONNECTION_PROPERTIES_HH

#include "fwd.hh"
#include "logical_time.hh"

namespace reactor {

enum ConnectionType { Normal, Delayed, Enclaved, Physical, DelayedEnclaved, PhysicalEnclaved, Plugin, Invalid };
struct ConnectionProperties {
  ConnectionType type_ = ConnectionType::Normal;
  Duration delay_{0};
  Environment* enclave_{nullptr};

  auto operator<(const ConnectionProperties& elem2) const noexcept -> bool {
    return (this->type_ < elem2.type_) || (this->type_ == elem2.type_ && this->delay_ < elem2.delay_);
  }

  auto operator==(const ConnectionProperties& elem2) const noexcept -> bool {
    return this->type_ == elem2.type_ && this->delay_ == elem2.delay_;
  }
};

} // namespace reactor

#endif // REACTOR_CPP_CONNECTION_PROPERTIES_HH
