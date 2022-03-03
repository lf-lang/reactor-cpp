Tracing reactor programms
---

Optionally reactor-cpp can be build with tracing support. This provides a
powerful tool for analyzing and debugging reactor applications.

## Required Dependencies

- [LTTng-ust](https://lttng.org) for recording traces
- [Babeltrace2](https://babeltrace.org/) with python bindings for converting traces
- Google Chrome or Chromium for viewing traces

## Build

1. Install LTTng as described [here](https://lttng.org/docs/#doc-installing-lttng)
2. Build and install Babeltrace2 including the python bindings as described [here](https://github.com/efficios/babeltrace/blob/stable-2.0/README.adoc)
3. Build reactor-cpp with tracing enabled:
```sh
mkdir build
cd build
cmake -DREACTOR_CPP_TRACE=ON..
make
```
4. Build your reactor application using reactor-cpp as normal (e.g. `make examples`)

## Usage

This directory contains a set of scripts that help in recording and converting
traces. Run `start_tracing.sh` to start and configure a lttng user space session. Then
start the application that you would like to trace
(e.g. `examples/hello/hello`). Use `stop_tracing.sh` to end the lttng session
after your application finished or when you want to abort tracing. This ensures
that all trace data is properly written to the files.

In order to view the trace, you have to convert it to a json file using
`ctf_to_json.py <lttng-session-dir>`. `<lttng-session-dir>` is the output
directory reported by `start_tracing.sh`. This produces a file
`trace.json`. Optionally, the default output file can be overridden using `-o`
or `--output`. If the conversion script fails, make sure that the generated
Babeltrace2 python bindings are within the `PYTHONPATH` and that `libabeltrace2.so` can be found
within `LD_LIBRARY_PATH`.

The trace can be viewed by opening `about://tracing` in Chrome (or
Chromium). There you can load the previously generated json file which produces
a visualization of the trace. In this visualization,  the top part (labeled *Execution*),
shows the physical time at which reactions execute. The bottom part shows individual reactors and logical times at which
actions are scheduled and reactions are triggered.

![Screenshot_20200512_165849](https://user-images.githubusercontent.com/6460123/81709144-fcb29a00-9471-11ea-9032-95cb6a368e98.png)

