/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/logging.hh"

namespace dear {
namespace log {

std::mutex BaseLogger::mutex;

}
}  // namespace dear
