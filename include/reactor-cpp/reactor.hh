/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Christian Menard
 */

#pragma once

#include <set>
#include <sstream>
#include <string>

#include "fwd.hh"
#include "time.hh"

namespace reactor {

class ReactorElement { //NOLINT
public:
    enum class Type { Action, Port, Reaction, Reactor };

private:
    const std::string name_;
    std::string fqn_;

    // The reactor owning this element
    Reactor* const container_;
    Environment* environment_;

    auto fqn_detail(std::stringstream& ss) const noexcept -> std::stringstream&;

public:
    ReactorElement(const std::string& name, Type type, Reactor* container);
    ReactorElement(const std::string& name, Type type, Environment* environment);
    virtual ~ReactorElement() = default;

    // not copyable, but movable
    ReactorElement(const ReactorElement&) = delete;
    ReactorElement(ReactorElement&&) = default;

    [[nodiscard]] auto container() const noexcept -> Reactor* {
        return container_;
    }

    [[nodiscard]] auto name() const noexcept -> const std::string& { return name_; }
    [[nodiscard]] auto fqn() const noexcept -> const std::string& { return fqn_; }
    [[nodiscard]] auto environment() const noexcept -> Environment* { return environment_; }

    [[nodiscard]] auto is_top_level() const noexcept -> bool {
        return this->container() == nullptr;
    }

    virtual void startup() = 0;
    virtual void shutdown() = 0;
};

class Reactor : public ReactorElement { //NOLINT
private:
    std::set<BaseAction*> actions_;
    std::set<BasePort*> inputs_;
    std::set<BasePort*> outputs_;
    std::set<Reaction*> reactions_;
    std::set<Reactor*> reactors_;

    void register_action(BaseAction* action);
    void register_port(BasePort* port);
    void register_reaction(Reaction* reaction);
    void register_reactor(Reactor* reactor);

public:
    Reactor(const std::string& name, Reactor* container);
    Reactor(const std::string& name, Environment* environment);
    ~Reactor() override = default;

    [[nodiscard]] auto actions() const noexcept -> const auto& { return actions_; }
    [[nodiscard]] auto inputs() const noexcept -> const auto& { return inputs_; }
    [[nodiscard]] auto outputs() const noexcept -> const auto& { return outputs_; }
    [[nodiscard]] auto reactions() const noexcept -> const auto& { return reactions_; }
    [[nodiscard]] auto reactors() const noexcept -> const auto& { return reactors_; }

    void startup() final;
    void shutdown() final;

    virtual void assemble() = 0;

    [[nodiscard]] static auto get_physical_time() noexcept -> TimePoint;
    [[nodiscard]] auto get_logical_time() const  noexcept -> TimePoint;
    [[nodiscard]] auto get_elapsed_logical_time() const noexcept -> Duration;
    [[nodiscard]] auto get_elapsed_physical_time() const noexcept -> Duration;

    friend ReactorElement;
};

}  // namespace reactor
