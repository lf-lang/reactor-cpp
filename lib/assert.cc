/*
 * Copyright (C) 2020 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/assert.hh"

namespace reactor {

auto ValidationError::build_message(const std::string& msg) noexcept -> std::string {
    std::stringstream ss;
    ss << "Validation Error! \"" << msg << "\"";
    return ss.str();
}

}  // namespace reactor
