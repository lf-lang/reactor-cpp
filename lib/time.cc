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

#include <ctime>
#include <iomanip>

namespace reactor {
inline namespace operators {

std::ostream& operator<<(std::ostream& os, TimePoint tp) {
  char buf[20];
  time_t time =
      std::chrono::system_clock::to_time_t(std::chrono::time_point_cast<std::chrono::system_clock::duration>(tp));
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&time));

  auto epoch = std::chrono::duration_cast<Duration>(tp.time_since_epoch());
  os << buf << '.' << std::setw(9) << std::setfill('0') << epoch.count() % 1'000'000'000UL;

  return os;
}

std::ostream& operator<<(std::ostream& os, std::chrono::seconds dur) {
  os << dur.count() << " secs";
  return os;
}
std::ostream& operator<<(std::ostream& os, std::chrono::milliseconds dur) {
  os << dur.count() << " msecs";
  return os;
}
std::ostream& operator<<(std::ostream& os, std::chrono::microseconds dur) {
  os << dur.count() << " usecs";
  return os;
}
std::ostream& operator<<(std::ostream& os, std::chrono::nanoseconds dur) {
  os << dur.count() << " nsecs";
  return os;
}

} // namespace operators

} // namespace reactor
