/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <chrono>

namespace dear {

using time_t = unsigned long long;  // at least 64 bit

// we assume 1ns clock granularity
constexpr time_t operator"" _ns(time_t x) { return x; }

constexpr time_t operator"" _us(time_t x) { return 1'000ull * x; }

constexpr time_t operator"" _ms(time_t x) { return 1'000'000ull * x; }

constexpr time_t operator"" _s(time_t x) { return 1'000'000'000ull * x; }

constexpr time_t operator"" _min(time_t x) { return 60'000'000'000ull * x; }

constexpr time_t operator"" _h(time_t x) { return 3'600'000'000'000ull * x; }

constexpr time_t operator"" _d(time_t x) { return 86'400'000'000'000ull * x; }

constexpr time_t operator"" _weeks(time_t x) {
  return 604'800'000'000'000ull * x;
}

time_t get_physical_timepoint();

void wait_until_physical_timepoint(time_t t);

void assert_physical_clock_resolution();

}  // namespace dear
