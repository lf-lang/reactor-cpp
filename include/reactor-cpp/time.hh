/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <chrono>
#include <iostream>

namespace reactor {

using TimePoint = std::chrono::time_point<std::chrono::system_clock,
                                          std::chrono::nanoseconds>;
using Duration = std::chrono::nanoseconds;

inline TimePoint get_physical_time() {
  return std::chrono::system_clock::now();
}

std::ostream& operator<<(std::ostream& os, TimePoint tp);

}  // namespace reactor
