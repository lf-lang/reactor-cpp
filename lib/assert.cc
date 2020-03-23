/*
 * Copyright (C) 2020 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/assert.hh"

namespace reactor {

std::string ValidationError::build_message(const std::string& msg) {
  std::stringstream ss;
  ss << "Validation Error! \"" << msg << "\"";
  return ss.str();
}

}  // namespace reactor
