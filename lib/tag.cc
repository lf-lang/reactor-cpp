/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "dear/tag.hh"

namespace dear {

bool operator==(const Tag& lhs, const Tag& rhs) {
  return lhs.time() == rhs.time() && lhs.micro_step() == rhs.micro_step();
}

bool operator<(const Tag& lhs, const Tag& rhs) {
  return lhs.time() < rhs.time() ||
         (lhs.time() == rhs.time() && lhs.micro_step() < rhs.micro_step());
}

Tag Tag::from_physical_time(time_t offset) {
  return Tag(get_physical_timepoint() + offset, 0);
}

Tag Tag::delay(time_t offset) {
  if (offset == 0) {
    return Tag(this->_time, this->_micro_step + 1);
  } else {
    return Tag(this->_time + offset, 0);
  }
}

}  // namespace dear
