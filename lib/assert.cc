/*
 * Copyright (C) 2020 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/assert.hh"

namespace reactor {

auto ValidationError::build_message(const std::string_view msg) noexcept -> std::string {
  std::stringstream string_stream;
  string_stream << "Validation Error! \"" << msg << "\"";
  return string_stream.str();
}

} // namespace reactor
