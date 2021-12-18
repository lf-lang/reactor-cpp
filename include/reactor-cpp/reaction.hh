/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <functional>
#include <set>

#include "reactor.hh"

namespace reactor {

class Reaction : public ReactorElement {
private:
    std::set<BaseAction*> _action_triggers;
    std::set<BaseAction*> _scheduable_actions;
    std::set<BasePort*> _port_triggers;
    std::set<BasePort*> _antidependencies;
    std::set<BasePort*> _dependencies;

    const int _priority;
    unsigned int _index;

    std::function<void(void)> body;

    Duration deadline{Duration::zero()};
    std::function<void(void)> deadline_handler{nullptr};

    void set_deadline_impl(Duration deadline, const std::function<void(void)>& handler);

public:
    Reaction(const std::string& name,
           int priority,
           Reactor* container,
           std::function<void(void)> body);

    ~Reaction() override = default;

    void declare_trigger(BaseAction* action);
    void declare_trigger(BasePort* port);
    void declare_schedulable_action(BaseAction* action);
    void declare_antidependency(BasePort* port);
    void declare_dependency(BasePort* port);

    [[nodiscard]] auto action_triggers() const noexcept -> const auto& {
          return _action_triggers;
    }

    [[nodiscard]] auto port_triggers() const noexcept -> const auto& {
          return _port_triggers;
    }

    [[maybe_unused]] [[nodiscard]] auto antidependencies() const noexcept -> const auto& {
          return _antidependencies;
    }

    [[nodiscard]] auto dependencies() const noexcept -> const auto& {
          return _dependencies;
    }

    [[maybe_unused]] [[nodiscard]] auto scheduable_actions() const noexcept -> const auto& {
          return _scheduable_actions;
    }

    [[nodiscard]] auto priority() const noexcept -> int { return _priority; }

    void startup() final {}
    void shutdown() final {}

    void trigger();

    template <class Dur>
    void set_deadline(Dur dl, const std::function<void(void)>& handler) {
        set_deadline_impl(std::chrono::duration_cast<Duration>(dl), handler);
    }
    [[nodiscard]] bool has_deadline() const {
        return deadline != Duration::zero();
    }

    void set_index(unsigned index);
    [[nodiscard]] auto index() const noexcept -> unsigned int {
        return _index;
    }
};

}  // namespace reactor
