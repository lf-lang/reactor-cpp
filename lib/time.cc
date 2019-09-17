/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/time.hh"

#include <cassert>

namespace dear {

time_t get_physical_timepoint() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

void assert_physical_clock_resolution() {
  double resolution = (double)std::chrono::high_resolution_clock::period::num /
                      std::chrono::high_resolution_clock::period::den;
  // we assume to have a clock resolution of at least 1ns
  assert(resolution <= 1.0e-9);
}

}  // namespace dear
