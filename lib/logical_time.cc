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

auto operator==(const Tag& lhs, const Tag& rhs) noexcept -> bool {
    return lhs.time_point() == rhs.time_point() &&
        lhs.micro_step() == rhs.micro_step();
}

bool operator<(const Tag& lhs, const Tag& rhs) {
    return lhs.time_point() < rhs.time_point() ||
        (lhs.time_point() == rhs.time_point() &&
        lhs.micro_step() < rhs.micro_step());
}

auto Tag::from_physical_time(TimePoint time_point) noexcept -> Tag {
    return Tag{time_point, 0};
}

auto Tag::from_logical_time(const LogicalTime& lt) noexcept -> Tag {
    return Tag{lt.time_point(), lt.micro_step()};
}

auto Tag::delay(Duration offset) const noexcept -> Tag {
    if (offset == Duration::zero()) {
        return Tag{this->time_point_, this->micro_step_ + 1};
    } else {
        return Tag{this->time_point_ + offset, 0};
    }
}

void LogicalTime::advance_to(const Tag& tag) {
    reactor_assert(*this < tag);
    _time_point = tag.time_point();
    _micro_step = tag.micro_step();
}

auto operator==(const LogicalTime& lhs, const Tag& rhs) noexcept -> bool {
    return lhs.time_point() == rhs.time_point() &&
        lhs.micro_step() == rhs.micro_step();
}

auto operator<(const LogicalTime& lhs, const Tag& rhs) noexcept -> bool {
    return lhs.time_point() < rhs.time_point() ||
        (lhs.time_point() == rhs.time_point() &&
        lhs.micro_step() < rhs.micro_step());
}

auto operator>(const LogicalTime& lhs, const Tag& rhs) noexcept -> bool {
    return lhs.time_point() > rhs.time_point() ||
        (lhs.time_point() == rhs.time_point() &&
        lhs.micro_step() > rhs.micro_step());
}

}  // namespace reactor
