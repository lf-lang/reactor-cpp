/*
 * Copyright (C) 2020 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "config.hh"

#ifdef REACTOR_CPP_TRACE

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER reactor_cpp

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "reactor-cpp/trace.hh"

#if !defined(_REACTOR_CPP_TRACE_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _REACTOR_CPP_TRACE_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
  reactor_cpp,
  reaction_execution_starts,
  TP_ARGS(
    int, worker_id_arg,
    const char*, reaction_name_arg
  ),
  TP_FIELDS(
    ctf_string(reaction_name, reaction_name_arg)
    ctf_integer(int, worker_id, worker_id_arg)
  )
)

TRACEPOINT_EVENT(
  reactor_cpp,
  reaction_execution_finishes,
  TP_ARGS(
    int, worker_id_arg,
    const char*, reaction_name_arg
  ),
  TP_FIELDS(
    ctf_string(reaction_name, reaction_name_arg)
    ctf_integer(int, worker_id, worker_id_arg)
  )
)

#endif /* _REACTOR_CPP_TRACE_H */

#include <lttng/tracepoint-event.h>

#else

// empty definition in case we compile without tracing
#define tracepoint(...)

#endif // REACTOR_CPP_TRACE
