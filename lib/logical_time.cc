/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/logical_time.hh"

#include "reactor-cpp/assert.hh"

#include <cassert>

namespace reactor {
bool operator==(const Tag& lhs, const Tag& rhs) {
  return lhs.time_point() == rhs.time_point() &&
         lhs.micro_step() == rhs.micro_step();
}

bool operator<(const Tag& lhs, const Tag& rhs) {
  return lhs.time_point() < rhs.time_point() ||
         (lhs.time_point() == rhs.time_point() &&
          lhs.micro_step() < rhs.micro_step());
}

Tag Tag::from_physical_time(TimePoint time_point) { return Tag(time_point, 0); }

Tag Tag::from_logical_time(const LogicalTime& lt) {
  return Tag(lt.time_point(), lt.micro_step());
}

Tag Tag::delay(Duration offset) const {
  if (offset == Duration::zero()) {
    return Tag(this->_time_point, this->_micro_step + 1);
  } else {
    return Tag(this->_time_point + offset, 0);
  }
}

void LogicalTime::advance_to(const Tag& tag) {
  assert(*this < tag);
  _time_point = tag.time_point();
  _micro_step = tag.micro_step();
}

bool operator==(const LogicalTime& lhs, const Tag& rhs) {
  return lhs.time_point() == rhs.time_point() &&
         lhs.micro_step() == rhs.micro_step();
}

bool operator<(const LogicalTime& lhs, const Tag& rhs) {
  return lhs.time_point() < rhs.time_point() ||
         (lhs.time_point() == rhs.time_point() &&
          lhs.micro_step() < rhs.micro_step());
}

bool operator>(const LogicalTime& lhs, const Tag& rhs) {
  return lhs.time_point() > rhs.time_point() ||
         (lhs.time_point() == rhs.time_point() &&
          lhs.micro_step() > rhs.micro_step());
}

}  // namespace reactor
