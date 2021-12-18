/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include "time.hh"

namespace reactor {

using mstep_t = unsigned long;  // at least 32 bit

class LogicalTime;

class Tag {
private:
    const TimePoint time_point_;
    const mstep_t micro_step_;

    Tag(const TimePoint& time_point, const mstep_t& micro_step)
      : time_point_{time_point}, micro_step_{micro_step} {}

public:
    // no default constructor, not assignable, but movable and copyable
    Tag() = delete;
    Tag& operator=(const Tag&) = delete;
    Tag(Tag&&) = default;
    Tag(const Tag&) = default;

    [[nodiscard]] auto time_point() const noexcept -> const TimePoint&{
        return time_point_;
    }
    [[nodiscard]] auto micro_step() const noexcept -> const mstep_t& {
        return micro_step_;
    }

    static auto from_physical_time(TimePoint time_point) noexcept -> Tag;
    static auto from_logical_time(const LogicalTime& lt) noexcept -> Tag;

    [[nodiscard]] auto delay(Duration offset = Duration::zero()) const noexcept -> Tag;
};

// define all the comparison operators
auto operator==(const Tag& lhs, const Tag& rhs) noexcept -> bool;
bool inline operator!=(const Tag& lhs, const Tag& rhs) { return !(lhs == rhs); }
bool operator<(const Tag& lhs, const Tag& rhs);
bool inline operator>(const Tag& lhs, const Tag& rhs) { return rhs < lhs; }
bool inline operator<=(const Tag& lhs, const Tag& rhs) { return !(lhs > rhs); }
bool inline operator>=(const Tag& lhs, const Tag& rhs) { return !(lhs < rhs); }

class LogicalTime {
private:
    TimePoint _time_point{};
    mstep_t _micro_step = 0;

public:
    void advance_to(const Tag& tag);

    [[nodiscard]] auto time_point() const noexcept -> TimePoint { return _time_point; }
    [[nodiscard]] auto micro_step() const noexcept -> mstep_t { return _micro_step; }
};

// c++20 starship operator will save us this mess
auto operator==(const LogicalTime& lt, const Tag& t) noexcept -> bool;
auto inline operator!=(const LogicalTime& lt, const Tag& t) noexcept -> bool { return !(lt == t);}
auto operator<(const LogicalTime& lt, const Tag& t) noexcept -> bool;
auto operator>(const LogicalTime& lt, const Tag& t) noexcept -> bool;
auto inline operator<=(const LogicalTime& lt, const Tag& t) noexcept -> bool { return !(lt > t);}
auto inline operator>=(const LogicalTime& lt, const Tag& t) noexcept -> bool { return !(lt < t);}
auto inline operator==(const Tag& t, const LogicalTime& lt) noexcept -> bool { return lt == t; }
auto inline operator!=(const Tag& t, const LogicalTime& lt) noexcept -> bool { return !(lt == t);}
auto inline operator<(const Tag& t, const LogicalTime& lt) noexcept -> bool { return lt > t; }
auto inline operator>(const Tag& t, const LogicalTime& lt) noexcept -> bool { return lt < t; }
auto inline operator<=(const Tag& t, const LogicalTime& lt) noexcept -> bool { return !(t > lt); }
auto inline operator>=(const Tag& t, const LogicalTime& lt) noexcept -> bool { return !(t < lt); }

}  // namespace reactor
