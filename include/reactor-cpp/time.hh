/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_TIME_HH
#define REACTOR_CPP_TIME_HH

#include "logging.hh"
#include <chrono>
#include <iostream>

namespace reactor {

using TimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;
using Duration = std::chrono::nanoseconds;

inline auto get_physical_time() noexcept -> TimePoint { return std::chrono::system_clock::now(); }


inline namespace operators {
auto operator<<(std::ostream &out_stream, reactor::TimePoint tp) -> std::ostream &;
auto operator<<(std::ostream &out_stream, std::chrono::seconds dur) -> std::ostream &;
auto operator<<(std::ostream &out_stream, std::chrono::milliseconds dur) -> std::ostream &;
auto operator<<(std::ostream &out_stream, std::chrono::microseconds dur) -> std::ostream &;
auto operator<<(std::ostream &out_stream, std::chrono::nanoseconds dur) -> std::ostream &;
} // namespace reactor::operators

} // namespace reactor

#endif // REACTOR_CPP_TIME_HH
