/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_ASSERT_HH
#define REACTOR_CPP_ASSERT_HH

#include "reactor-cpp/config.hh"
#include "reactor-cpp/fwd.hh"

#include <cassert>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef REACTOR_CPP_VALIDATE
constexpr bool runtime_validation = true;
#else
constexpr bool runtime_validation = false;
#endif

#ifdef NDEBUG
constexpr bool runtime_assertion = false;
#else
constexpr bool runtime_assertion = true;
#endif

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define reactor_assert(x) assert(x)

#ifdef REACTOR_CPP_USE_BACKTRACE

// NOLINTNEXTLINE
#include REACTOR_CPP_BACKTRACE_HEADER
#include <array>
#include <iostream>

namespace reactor {

constexpr std::size_t MAX_TRACE_SIZE{16};

inline void print_backtrace() {
  std::array<void*, MAX_TRACE_SIZE> trace{nullptr};
  int size = backtrace(trace.data(), MAX_TRACE_SIZE);
  char** messages = backtrace_symbols(trace.data(), size);
  for (int i{0}; i < size; i++) {
    std::cerr << "[backtrace] " << messages[i] << '\n'; // NOLINT
  }
}

} // namespace reactor
#else
namespace reactor {
inline void print_backtrace() {}
} // namespace reactor
#endif // REACTOR_CPP_BACKTRACE_SUPPORT

namespace reactor {

class ValidationError : public std::runtime_error {
private:
  static auto build_message(std::string_view msg) noexcept -> std::string;

public:
  explicit ValidationError(const std::string_view msg)
      : std::runtime_error(build_message(msg)) {}
};

constexpr inline void validate([[maybe_unused]] bool condition, [[maybe_unused]] const std::string_view message) {
  if constexpr (runtime_validation) { // NOLINT
    if (!condition) {
      print_backtrace();
      throw ValidationError(message);
    }
  }
}

template <typename E> constexpr auto extract_value(E enum_value) -> typename std::underlying_type<E>::type {
  return static_cast<typename std::underlying_type<E>::type>(enum_value);
}

void assert_phase([[maybe_unused]] const ReactorElement* ptr, [[maybe_unused]] Phase phase);

} // namespace reactor

#endif // REACTOR_CPP_ASSERT_HH
