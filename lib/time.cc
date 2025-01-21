/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

// tell MSCV not to worry about the potential unsafe use of localtime
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

#include "reactor-cpp/time.hh"

#include <array>
#include <ctime>
#include <iomanip>

namespace reactor {

constexpr std::size_t TIME_TO_STR_BUFFER_SIZE{20};
constexpr std::size_t NANOSECONDS_IN_ONE_SECOND{1'000'000'000UL};
constexpr std::size_t NANOSECOND_DIGITS{9};

inline namespace operators {

auto operator<<(std::ostream& os, const TimePoint tp) -> std::ostream& {
  std::array<char, TIME_TO_STR_BUFFER_SIZE> buf{};
  const time_t time =
      std::chrono::system_clock::to_time_t(std::chrono::time_point_cast<std::chrono::system_clock::duration>(tp));
  const auto res = std::strftime(buf.data(), sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&time));
  const auto epoch = std::chrono::duration_cast<Duration>(tp.time_since_epoch());

  if (res != 0) {
    os << buf.data() << '.' << std::setw(NANOSECOND_DIGITS) << std::setfill('0')
       << epoch.count() % NANOSECONDS_IN_ONE_SECOND;
  } else {
    os << "[INVALID TIME]";
  }

  return os;
}

auto operator<<(std::ostream& os, const std::chrono::seconds dur) -> std::ostream& {
  os << dur.count() << " secs";
  return os;
}
auto operator<<(std::ostream& os, const std::chrono::milliseconds dur) -> std::ostream& {
  os << dur.count() << " msecs";
  return os;
}
auto operator<<(std::ostream& os, const std::chrono::microseconds dur) -> std::ostream& {
  os << dur.count() << " usecs";
  return os;
}
auto operator<<(std::ostream& os, const std::chrono::nanoseconds dur) -> std::ostream& {
  os << dur.count() << " nsecs";
  return os;
}

} // namespace operators

} // namespace reactor
