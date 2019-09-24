/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/input.hh"
#include "dear/output.hh"

#include <cassert>

namespace dear {

void BaseInput::bind_base(BaseOutput* output) {
  assert(!is_bound());
  assert(this->container()->container() == output->container()->container());
  assert(output != nullptr);
  binding = output;
}

}  // namespace dear
