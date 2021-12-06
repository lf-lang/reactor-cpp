/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#ifdef REACTOR_CPP_VALIDATE
constexpr bool runtime_validation = REACTOR_CPP_VALIDATE;
#else
constexpr bool runtime_validation = false;
#endif

#ifdef NDEBUG
constexpr bool runtime_assertion = NDEBUG;
#else
constexpr bool runtime_assertion = false;
#endif

// macro for silencing unused warnings my the compiler
#define UNUSED(expr) do { (void)(expr); } while (0)

#include <cassert>
#include <sstream>
#include <stdexcept>
#include <string>

namespace reactor {

class ValidationError : public std::runtime_error {
 private:
  static std::string build_message(const std::string& msg);

 public:
  explicit ValidationError(const std::string& msg)
      : std::runtime_error(build_message(msg)) {}
};

constexpr inline void validate(bool condition, const std::string& message) {
  if constexpr (runtime_validation && !condition) {
    throw ValidationError(message);
  }
}

constexpr inline void toggle_assert(bool condition) {
  if constexpr (runtime_assertion){
    UNUSED(condition);
    assert(condition);
  }
}

}  // namespace reactor
