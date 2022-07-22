/*
 * Copyright (C) 2022 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#include "reactor-cpp/base_scheduler.hh"

#include "reactor-cpp/action.hh"
#include "reactor-cpp/assert.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/fwd.hh"
#include "reactor-cpp/logging.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/trace.hh"

namespace reactor {

BaseScheduler::BaseScheduler(Environment* env)
    : environment_(env)
    , using_workers_(env->num_workers() > 1) {}

void BaseScheduler::schedule_sync(const Tag& tag, BaseAction* action, std::function<void(void)> pre_handler) {
  reactor_assert(logical_time_ < tag);
  // TODO verify that the action is indeed allowed to be scheduled by the
  // current reaction
  log::Debug() << "Schedule action " << action->fqn() << (action->is_logical() ? " synchronously " : " asynchronously ")
               << " with tag [" << tag.time_point() << ", " << tag.micro_step() << "]";
  {
    auto unique_lock =
        using_workers_ ? std::unique_lock<std::mutex>(lock_event_queue_) : std::unique_lock<std::mutex>();

    tracepoint(reactor_cpp, schedule_action, action->container()->fqn(), action->name(), tag); // NOLINT

    // create a new event map or retrieve the existing one
    auto emplace_result = event_queue_.try_emplace(tag, EventMap());
    auto& event_map = emplace_result.first->second;

    // insert the new event
    event_map[action] = std::move(pre_handler);
  }
}

void BaseScheduler::schedule_async(const Tag& tag, BaseAction* action, std::function<void(void)> pre_handler) {
  std::lock_guard<std::mutex> lock_guard(scheduling_mutex_);
  schedule_sync(tag, action, std::move(pre_handler));
  cv_schedule_.notify_one();
}
} // namespace reactor
