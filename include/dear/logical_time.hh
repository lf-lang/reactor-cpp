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

  Tag delay(time_t offset = 0) const;
};

// define all the comparison operators
bool operator==(const Tag& lhs, const Tag& rhs);
bool inline operator!=(const Tag& lhs, const Tag& rhs) { return !(lhs == rhs); }
bool operator<(const Tag& lhs, const Tag& rhs);
bool inline operator>(const Tag& lhs, const Tag& rhs) { return rhs < lhs; }
bool inline operator<=(const Tag& lhs, const Tag& rhs) { return !(lhs > rhs); }
bool inline operator>=(const Tag& lhs, const Tag& rhs) { return !(lhs < rhs); }

class LogicalTime {
 private:
  time_t _time{0};
  mstep_t _micro_step{0};

 public:
  void advance_to(const Tag& tag);

  time_t time() const { return _time; }
  mstep_t micro_step() const { return _micro_step; }
};

bool operator==(const LogicalTime& lt, const Tag& t);
bool inline operator!=(const LogicalTime& lt, const Tag& t) { return !(lt == t); }
bool operator<(const LogicalTime& lt, const Tag& t);
bool operator>(const LogicalTime& lt, const Tag& t);
bool inline operator<=(const LogicalTime& lt, const Tag& t) { return !(lt > t); }
bool inline operator>=(const LogicalTime& lt, const Tag& t) { return !(lt < t); }

bool inline operator==(const Tag& t, const LogicalTime& lt) { return lt == t; }
bool inline operator!=(const Tag& t, const LogicalTime& lt) { return !(lt == t); }
bool inline operator<(const Tag& t, const LogicalTime& lt) { return lt > t; }
bool inline operator>(const Tag& t, const LogicalTime& lt) { return lt < t; }
bool inline operator<=(const Tag& t, const LogicalTime& lt) { return !(t > lt); }
bool inline operator>=(const Tag& t, const LogicalTime& lt) { return !(t < lt); }

}  // namespace dear
