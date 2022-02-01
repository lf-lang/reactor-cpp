/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#ifndef REACTOR_CPP_ASSERT_HH
#define REACTOR_CPP_ASSERT_HH

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

#include "environment.hh"

#include <cassert>
#include <sstream>
#include <stdexcept>
#include <string>

namespace reactor {
using EnvPhase = Environment::Phase;

class ValidationError : public std::runtime_error {
private:
  static auto build_message(const std::string& msg) noexcept -> std::string;

public:
  explicit ValidationError(const std::string& msg)
      : std::runtime_error(build_message(msg)) {}
};

constexpr inline void validate([[maybe_unused]] bool condition, [[maybe_unused]] const std::string& message) {
  if constexpr (runtime_validation && !condition) { // NOLINT
    throw ValidationError(message);
  }
}

constexpr inline void reactor_assert([[maybe_unused]] bool condition) {
  if constexpr (runtime_assertion) { // NOLINT
    assert(condition); //NOLINT
  }
}

template<typename E>
constexpr auto to_underlying_type(E enum_value) -> typename std::underlying_type<E>::type {
  return static_cast<typename std::underlying_type<E>::type>(enum_value);
}

inline void assert_phase([[maybe_unused]] const ReactorElement* ptr, [[maybe_unused]] EnvPhase phase) {
  if constexpr (runtime_assertion) { // NOLINT
    if (ptr->environment()->phase() != phase) {
      auto enum_value_to_name = [](EnvPhase phase) -> std::string {
        const std::map<EnvPhase, std::string> conversation_map = { //NOLINT
            {EnvPhase::Construction, "Construction"},
            {EnvPhase::Assembly, "Assembly"},
            {EnvPhase::Startup, "Startup"},
            {EnvPhase::Execution, "Execution"},
            {EnvPhase::Shutdown, "Shutdown"},
            {EnvPhase::Deconstruction, "Deconstruction"}
        };
        // in C++20 use .contains()
        return (conversation_map.find(phase) != conversation_map.end()) ? "Unknown Phase: Value: "
            + std::to_string(to_underlying_type(phase)) : conversation_map.at(phase);
      };
      // C++20 std::format
      throw ValidationError("Expected Phase: " + enum_value_to_name(phase) +
                            " Current Phase: " + enum_value_to_name(ptr->environment()->phase()));
    }
  }
}
} // namespace reactor

#endif // REACTOR_CPP_ASSERT_HH
