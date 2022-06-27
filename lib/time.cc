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

constexpr size_t TIME_TO_STR_BUFFER_SIZE{20};

inline namespace operators {

auto operator<<(std::ostream& os, TimePoint tp) -> std::ostream& {
  std::array<char, TIME_TO_STR_BUFFER_SIZE> buf{};
  time_t time =
      std::chrono::system_clock::to_time_t(std::chrono::time_point_cast<std::chrono::system_clock::duration>(tp));
  std::strftime(buf.data(), sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&time));

  auto epoch = std::chrono::duration_cast<Duration>(tp.time_since_epoch());
  os << buf.data() << '.' << std::setw(9) << std::setfill('0') << epoch.count() % 1'000'000'000UL; // NOLINT

  return os;
}

auto operator<<(std::ostream& os, std::chrono::seconds dur) -> std::ostream& {
  os << dur.count() << " secs";
  return os;
}
auto operator<<(std::ostream& os, std::chrono::milliseconds dur) -> std::ostream& {
  os << dur.count() << " msecs";
  return os;
}
auto operator<<(std::ostream& os, std::chrono::microseconds dur) -> std::ostream& {
  os << dur.count() << " usecs";
  return os;
}
auto operator<<(std::ostream& os, std::chrono::nanoseconds dur) -> std::ostream& {
  os << dur.count() << " nsecs";
  return os;
}

} // namespace operators

} // namespace reactor
