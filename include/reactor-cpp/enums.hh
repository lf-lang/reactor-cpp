//
// Created by revol-xut on 5/31/23.
//

#ifndef REACTOR_CPP_ENUMS_HH
#define REACTOR_CPP_ENUMS_HH

#include "logical_time.hh"

namespace reactor {

enum ConnectionType { Normal, Delayed, Enclaved, Physical, DelayedEnclaved, PhysicalEnclaved, Plugin };
struct ConnectionProperties {
  ConnectionType type_ = ConnectionType::Normal;
  Duration delay_{0};
};

} // namespace reactor

#endif // REACTOR_CPP_ENUMS_HH
