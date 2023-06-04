/*
 * Copyright (C) 2020 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/reactor_element.hh"

namespace reactor {

auto ValidationError::build_message(const std::string_view msg) noexcept -> std::string {
  std::stringstream string_stream;
  string_stream << "Validation Error! \"" << msg << "\"";
  return string_stream.str();
}

void assert_phase([[maybe_unused]] const ReactorElement* ptr, [[maybe_unused]] Phase phase) {
  if constexpr (runtime_assertion) { // NOLINT
    if (ptr->environment()->phase() != phase) {
      auto enum_value_to_name = [](Phase phase) -> std::string {
        const std::map<Phase, std::string> conversation_map = {
            // NOLINT
            {Phase::Construction, "Construction"}, {Phase::Assembly, "Assembly"},
            {Phase::Startup, "Startup"},           {Phase::Execution, "Execution"},
            {Phase::Shutdown, "Shutdown"},         {Phase::Deconstruction, "Deconstruction"}};
        // in C++20 use .contains()
        if (conversation_map.find(phase) != std::end(conversation_map)) {
          return conversation_map.at(phase);
        }
        return "Unknown Phase: Value: " + std::to_string(extract_value(phase));
      };
#ifdef __linux__
      print_debug_backtrace();
#endif

      // C++20 std::format
      throw ValidationError("Expected Phase: " + enum_value_to_name(phase) +
                            " Current Phase: " + enum_value_to_name(ptr->environment()->phase()));
    }
  }
}

} // namespace reactor
