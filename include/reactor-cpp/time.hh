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

inline auto get_physical_time() noexcept -> TimePoint {
    return std::chrono::system_clock::now();
}

inline namespace operators {

std::ostream& operator<<(std::ostream& os, TimePoint tp);

std::ostream& operator<<(std::ostream& os, std::chrono::seconds dur);
std::ostream& operator<<(std::ostream& os, std::chrono::milliseconds dur);
std::ostream& operator<<(std::ostream& os, std::chrono::microseconds dur);
std::ostream& operator<<(std::ostream& os, std::chrono::nanoseconds dur);

}  // namespace operators

}  // namespace reactor
