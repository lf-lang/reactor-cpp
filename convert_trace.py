#!/usr/bin/env python3

# Copyright (C) 2020 TU Dresden
# All rights reserved.
#
# Authors:
#   Christian Menard


import bt2
import json
import sys


def main():
    # Find the `ctf` plugin (shipped with Babeltrace 2).
    ctf_plugin = bt2.find_plugin('ctf')

    # Get the `source.ctf.fs` component class from the plugin.
    fs_cc = ctf_plugin.source_component_classes['fs']

    # Create a trace collection message iterator, instantiating a single
    # `source.ctf.fs` component class with the `inputs` initialization
    # parameter set to open a single CTF trace.
    msg_it = bt2.TraceCollectionMessageIterator(bt2.ComponentSpec(fs_cc, {
        # Get the CTF trace path from the first command-line argument.
        'inputs': [sys.argv[1]],
    }))

    # keep a list of events to dump later to JSON
    trace_events = []

    # Iterate the trace messages.
    for msg in msg_it:
        # `bt2._EventMessageConst` is the Python type of an event message.
        if type(msg) is bt2._EventMessageConst:
            event = msg.event

            if (event.name == "reactor_cpp:reaction_execution_starts"):
                trace_events.append(reaction_execution_starts_to_dict(msg))
            elif (event.name == "reactor_cpp:reaction_execution_finishes"):
                trace_events.append(reaction_execution_finishes_to_dict(msg))

            # Print event's name.
            print("%s: %d -> %s" % (event.name,
                                    event["worker_id"],
                                    event["reaction_name"]))

    # add some metadata
    configure_process_name(trace_events, 0, "Execution")
    configure_thread_name(trace_events, 0, 0, "Scheduler")
    for i in range(1, 17):
        configure_thread_name(trace_events, 0, i, "Worker %d" % i)

    data = {
        "traceEvents": trace_events,
        "displayTimeUnit": "ns",
    }
    with open('trace.json', 'w') as outfile:
        json.dump(data, outfile, indent=2)


def configure_process_name(events, pid, name):
    events.append({
        "name": "process_name",
        "ph": "M",
        "pid": pid,
        "args": {
            "name": name
        }
    })


def configure_thread_name(events, pid, tid, name):
    events.append({
        "name": "thread_name",
        "ph": "M",
        "pid": pid,
        "tid": tid,
        "args": {
            "name": name
        }
    })


def get_timestamp_us(msg):
    timestamp_ns = msg.default_clock_snapshot.ns_from_origin
    return timestamp_ns / 1000.0


def reaction_execution_starts_to_dict(msg):
    event = msg.event
    return {
        "name": str(event["reaction_name"]),
        "cat": "Execution",
        "ph": "B",
        "ts": get_timestamp_us(msg),
        "pid": 0,
        "tid": int(event["worker_id"]),
    }


def reaction_execution_finishes_to_dict(msg):
    event = msg.event
    return {
        "name": str(event["reaction_name"]),
        "cat": "Execution",
        "ph": "E",
        "ts": get_timestamp_us(msg),
        "pid": 0,
        "tid": int(event["worker_id"]),
    }


if(__name__ == "__main__"):
    main()
