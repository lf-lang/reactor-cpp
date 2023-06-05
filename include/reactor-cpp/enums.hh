/*
 * Copyright (C) 2023 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_ENUMS_HH
#define REACTOR_CPP_ENUMS_HH

#include "fwd.hh"
#include "logical_time.hh"

namespace reactor {

enum ConnectionType { Normal, Delayed, Enclaved, Physical, DelayedEnclaved, PhysicalEnclaved, Plugin, Invalid };
struct ConnectionProperties {
  ConnectionType type_ = ConnectionType::Normal;
  Duration delay_{0};
  Environment* enclave_{nullptr};
};

} // namespace reactor

#endif // REACTOR_CPP_ENUMS_HH
