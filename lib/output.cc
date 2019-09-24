/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/output.hh"
#include "dear/input.hh"

#include <cassert>

namespace dear {

void BaseOutput::bind_base(BaseInput* input) {
  assert(input != nullptr);
  assert(this->container()->container() == input->container()->container());
  auto result = bindings.insert(input);
  assert(result.second);
}

}  // namespace dear
