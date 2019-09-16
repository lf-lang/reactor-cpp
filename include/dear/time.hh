/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

namespace dear {

using time_t = unsigned long long;  // at least 64 bit

// we assume 1ns clock granularity
constexpr time_t operator"" _ns(time_t x) {
  return x;
}

constexpr time_t operator"" _us(time_t x) {
  return 1'000ull * x;
}

constexpr time_t operator"" _ms(time_t x) {
  return 1'000'000ull * x;
}

constexpr time_t operator"" _s(time_t x) {
  return 1'000'000'000ull * x;
}

constexpr time_t operator"" _min(time_t x) {
  return 60'000'000'000ull * x;
}

constexpr time_t operator"" _hr(time_t x) {
  return 3'600'000'000'000ull * x;
}

}  // namespace dear
