/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/logging.hh"

namespace reactor {
namespace log {

std::mutex BaseLogger<true>::mutex;

}
}  // namespace reactor
