/*
 * Copyright (C) 2020 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/validate.hh"

namespace reactor {

std::string ValidationError::build_message(const std::string& msg) {
  std::stringstream ss;
  ss << "Validation Error! \"" << msg << "\"";
  return ss.str();
}

void validate(bool condition, const std::string& message) {
  if (!condition) {
    throw ValidationError(message);
  }
}

}  // namespace reactor
