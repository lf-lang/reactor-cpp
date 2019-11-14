/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/time.hh"

#include <thread>

#include "reactor-cpp/assert.hh"

namespace reactor {

time_t get_physical_timepoint() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

void wait_until_physical_timepoint(time_t t) {
  std::chrono::nanoseconds dur(t);
  std::chrono::time_point<std::chrono::system_clock> tp(dur);
  std::this_thread::sleep_until(tp);
}

void assert_physical_clock_resolution() {
  double resolution = (double)std::chrono::high_resolution_clock::period::num /
                      std::chrono::high_resolution_clock::period::den;
  // we assume to have a clock resolution of at least 1ns
  ASSERT(resolution <= 1.0e-9);
}

}  // namespace reactor
