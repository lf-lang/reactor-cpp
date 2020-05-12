#!/bin/bash

lttng create reactor-cpp-session
lttng enable-event --userspace "reactor_cpp:*"
lttng start
