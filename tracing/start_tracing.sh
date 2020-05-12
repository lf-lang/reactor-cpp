#!/bin/bash

lttng create my-user-space-session
lttng enable-event --userspace "reactor_cpp:*"
lttng start
