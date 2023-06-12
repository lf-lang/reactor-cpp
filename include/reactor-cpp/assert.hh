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

#ifdef __linux__
#include <execinfo.h>
#include <unistd.h>
#endif

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

namespace reactor {

class ValidationError : public std::runtime_error {
private:
  static auto build_message(std::string_view msg) noexcept -> std::string;

public:
  explicit ValidationError(const std::string_view msg)
      : std::runtime_error(build_message(msg)) {}
};

#ifdef __linux__
constexpr std::size_t MAX_STACK_SIZE{10};

inline void print_debug_backtrace() {
  void* array[10]; // NOLINT
  // get void*'s for all entries on the stack
  int size = backtrace((void**)array, MAX_STACK_SIZE);
  backtrace_symbols_fd((void**)array, size, STDERR_FILENO);
}
#endif

constexpr inline void validate([[maybe_unused]] bool condition, [[maybe_unused]] const std::string_view message) {
  if constexpr (runtime_validation) { // NOLINT
    if (!condition) {
#ifdef __linux__
      print_debug_backtrace();
#endif
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
