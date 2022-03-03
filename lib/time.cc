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

#include "reactor-cpp/logging.hh"
#include "reactor-cpp/time.hh"

#include <ctime>
#include <iomanip>

//namespace reactor {
//inline namespace operators {

auto operator<<([[maybe_unused]]std::ostream &output_stream, [[maybe_unused]]reactor::TimePoint time_point) noexcept -> std::ostream &{
  constexpr std::size_t field_size = 20;
  constexpr int width = 9;
  constexpr auto max_value = 1'000'000'000UL;

  char buffer[field_size];
  time_t time = std::chrono::system_clock::to_time_t(
      std::chrono::time_point_cast<std::chrono::system_clock::duration>(time_point));
  std::strftime(buffer, field_size, "%Y-%m-%d %H:%M:%S", std::localtime(&time));

  auto epoch = std::chrono::duration_cast<reactor::Duration>(time_point.time_since_epoch());
  output_stream << buffer << '.' << std::setw(width) << std::setfill('0')
     << epoch.count() % max_value;

  return output_stream;
}

auto operator<<(std::ostream &out_stream, std::chrono::seconds dur) noexcept -> std::ostream & {
  out_stream << dur.count() << " secs";
  return out_stream;
}
auto operator<<(std::ostream &out_stream, std::chrono::milliseconds dur) noexcept -> std::ostream & {
  out_stream << dur.count() << " msecs";
  return out_stream;
}
auto operator<<(std::ostream &out_stream, std::chrono::microseconds dur) noexcept -> std::ostream & {
  out_stream << dur.count() << " usecs";
  return out_stream;
}
auto operator<<(std::ostream &out_stream, std::chrono::nanoseconds dur) noexcept -> std::ostream & {
  out_stream << dur.count() << " nsecs";
  return out_stream;
}

//} // namespace operators

//} // namespace reactor
