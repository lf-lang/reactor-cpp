/*
 * Copyright (C) 2020 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include <string>

#include "config.hh" //NOLINT
#include "logical_time.hh"

#ifdef REACTOR_CPP_TRACE_HH

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER reactor_cpp

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "reactor-cpp/trace.hh"

// LTTng requires this header to be included multiple times. Therfore, we cannot
// use `#praga once`, unfortunantely.
#ifndef _REACTOR_CPP_TRACE_H
namespace reactor {
constexpr bool tracing_enabled = true;
}
#endif

#if !defined(_REACTOR_CPP_TRACE_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _REACTOR_CPP_TRACE_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(reactor_cpp, reaction_execution_starts,
                 TP_ARGS(int, worker_id_arg, const std::string&, reaction_name_arg),
                 TP_FIELDS(ctf_string(reaction_name, reaction_name_arg.c_str())
                               ctf_integer(int, worker_id, worker_id_arg)))

TRACEPOINT_EVENT(reactor_cpp, reaction_execution_finishes,
                 TP_ARGS(int, worker_id_arg, const std::string&, reaction_name_arg),
                 TP_FIELDS(ctf_string(reaction_name, reaction_name_arg.c_str())
                               ctf_integer(int, worker_id, worker_id_arg)))

TRACEPOINT_EVENT(
    reactor_cpp, schedule_action,
    TP_ARGS(const std::string&, reactor_name_arg, const std::string&, action_name_arg, const reactor::Tag&, tag_arg),
    TP_FIELDS(ctf_string(reactor_name, reactor_name_arg.c_str()) ctf_string(action_name, action_name_arg.c_str())
                  ctf_integer(unsigned long, timestamp_ns, tag_arg.time_point().time_since_epoch().count())
                      ctf_integer(unsigned, timestamp_microstep, tag_arg.micro_step())))

TRACEPOINT_EVENT(reactor_cpp, trigger_reaction,
                 TP_ARGS(const std::string&, reactor_name_arg, const std::string&, reaction_name_arg,
                         const reactor::LogicalTime&, tag_arg),
                 TP_FIELDS(ctf_string(reactor_name, reactor_name_arg.c_str()) ctf_string(reaction_name,
                                                                                         reaction_name_arg.c_str())
                               ctf_integer(unsigned long, timestamp_ns, tag_arg.time_point().time_since_epoch().count())
                                   ctf_integer(unsigned, timestamp_microstep, tag_arg.micro_step())))

#endif /* _REACTOR_CPP_TRACE_H */

#include <lttng/tracepoint-event.h>

#else

#ifndef _REACTOR_CPP_TRACE_H
#define REACTOR_CPP_TRACE_H

namespace reactor {
constexpr bool tracing_enabled = false;
} // namespace reactor

// empty definition in case we compile without tracing
#define tracepoint(...)

#endif // _REACTOR_CPP_TRACE_H

#endif // REACTOR_CPP_TRACE_HH
