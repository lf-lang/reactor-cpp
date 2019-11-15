/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/logical_time.hh"

#include "reactor-cpp/assert.hh"

namespace reactor {

bool operator==(const Tag& lhs, const Tag& rhs) {
  return lhs.time() == rhs.time() && lhs.micro_step() == rhs.micro_step();
}

bool operator<(const Tag& lhs, const Tag& rhs) {
  return lhs.time() < rhs.time() ||
         (lhs.time() == rhs.time() && lhs.micro_step() < rhs.micro_step());
}

Tag Tag::from_physical_time(time_t time_point) {
  return Tag(time_point, 0);
}

Tag Tag::from_logical_time(const LogicalTime& lt) {
  return Tag(lt.time(), lt.micro_step());
}

Tag Tag::delay(time_t offset) const {
  if (offset == 0) {
    return Tag(this->_time, this->_micro_step + 1);
  } else {
    return Tag(this->_time + offset, 0);
  }
}

void LogicalTime::advance_to(const Tag& tag) {
  ASSERT(*this < tag);
  _time = tag.time();
  _micro_step = tag.micro_step();
}

bool operator==(const LogicalTime& lhs, const Tag& rhs) {
  return lhs.time() == rhs.time() && lhs.micro_step() == rhs.micro_step();
}

bool operator<(const LogicalTime& lhs, const Tag& rhs) {
  return lhs.time() < rhs.time() ||
         (lhs.time() == rhs.time() && lhs.micro_step() < rhs.micro_step());
}

bool operator>(const LogicalTime& lhs, const Tag& rhs) {
  return lhs.time() > rhs.time() ||
         (lhs.time() == rhs.time() && lhs.micro_step() > rhs.micro_step());
}

}  // namespace reactor
