/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_REACTOR_CPP_HH
#define REACTOR_CPP_REACTOR_CPP_HH

// include everything that is needed to use reactor-cpp
#include "action.hh"
#include "connection.hh"
#include "connection_endpoint.hh"
// TODO only include this if ros is acutally requested (and therefore must be installed)
#include "ros2_connection_endpoint.hh"
#include "environment.hh"
#include "logging.hh"
#include "logical_time.hh"
#include "multiport.hh"
#include "port.hh"
#include "reaction.hh"
#include "reactor.hh"
#include "time.hh"

#endif // REACTOR_CPP_REACTOR_CPP_HH
