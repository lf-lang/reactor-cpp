/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include "time.hh"

namespace dear {

using mstep_t = unsigned long;  // at least 32 bit

class Tag {
 private:
  const time_t _time;
  const mstep_t _micro_step;

  Tag(time_t time, mstep_t micro_step) : _time{time}, _micro_step{micro_step} {}

 public:
  // no default constructor, not assignable, but movable and copyable
  Tag() = delete;
  Tag& operator=(const Tag&) = delete;
  Tag(Tag&&) = default;
  Tag(const Tag&) = default;

  time_t time() const { return _time; }
  mstep_t micro_step() const { return _micro_step; }

  static Tag from_physical_time(time_t offset = 0);

  Tag delay(time_t offset = 0);
};

// define all the comparison operators
bool operator==(const Tag& lhs, const Tag& rhs);
bool operator!=(const Tag& lhs, const Tag& rhs) { return !(lhs == rhs); }
bool operator<(const Tag& lhs, const Tag& rhs);
bool operator>(const Tag& lhs, const Tag& rhs) { return rhs < lhs; }
bool operator<=(const Tag& lhs, const Tag& rhs) { return !(lhs > rhs); }
bool operator>=(const Tag& lhs, const Tag& rhs) { return !(lhs < rhs); }

}  // namespace dear
