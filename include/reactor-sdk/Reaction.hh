#pragma once

#include "reactor-cpp/reactor-cpp.hh"
#include "ReactionBase.hh"
#include "Reactor.hh"

namespace sdk
{

template <typename Fn, typename InputTuple, typename DependencyTuple, typename OutputTuple>
class Reaction;

template <typename T, template <typename...> class Template>
struct is_specialization : std::false_type
{
};

template <typename... Args, template <typename...> class Template>
struct is_specialization<Template<Args...>, Template> : std::true_type
{
};

template <typename T, template <typename...> class Template>
inline constexpr bool is_specialization_v = is_specialization<T, Template>::value;

// fix for gcc < 13
template <typename T>
constexpr bool templated_false = false;

template <typename InputTuple, typename DependencyTuple, typename OutputTuple>
class ReactionOutput: public ReactionBase
{
private:
    InputTuple input_triggers;
    DependencyTuple dependencies;
    OutputTuple output_triggers;

public:
    explicit ReactionOutput(std::string name, Reactor *parent, InputTuple inputs, DependencyTuple deps, OutputTuple outputs)
        : ReactionBase (name, parent), input_triggers(std::move(inputs)), dependencies(std::move(deps)), output_triggers(std::move(outputs)) {}
    ~ReactionOutput() {}

    template <typename Fn>
    Reaction<Fn, InputTuple, DependencyTuple, OutputTuple> &function(Fn func)
    {
        if constexpr (std::is_bind_expression<Fn>::value) {
        }
        else if (sizeof(func) != sizeof(void*)) {
            reactor::log::Error() << "Reactor: " << reactor->fqn() << " Reaction: " << name << " Accesses variables outside of its scope";
            exit(EXIT_FAILURE);
        }

        auto ReactionRef = std::make_shared<Reaction<Fn, InputTuple, DependencyTuple, OutputTuple>> (name, reactor, std::move(input_triggers), std::move(dependencies), std::move(output_triggers), std::forward<Fn>(func));
        ReactionRef->execute();
        return *ReactionRef;
    }
};

template <typename InputTuple, typename DependencyTuple>
class ReactionDependency: public ReactionBase
{
private:
    InputTuple input_triggers;
    DependencyTuple dependencies;

public:
    explicit ReactionDependency(std::string name, Reactor *parent, InputTuple inputs, DependencyTuple deps)
        : ReactionBase (name, parent), input_triggers(inputs), dependencies(std::move(deps)) {}
    ~ReactionDependency() {}

    template <typename... Outputs>
    ReactionOutput<InputTuple, DependencyTuple, std::tuple<Outputs...>> &effects(Outputs&&... outputs)
    {
        auto output_tuple = std::make_tuple(outputs...);
        auto ReactionOutputRef = std::make_shared<ReactionOutput<InputTuple, DependencyTuple, std::tuple<Outputs...>>> (name, reactor, std::move(input_triggers), std::move(dependencies), std::move(output_tuple));
        next = ReactionOutputRef;
        return *ReactionOutputRef;
    }
};

template <typename InputTuple>
class ReactionInput: public ReactionBase
{
private:
    InputTuple input_triggers;

public:
    explicit ReactionInput(std::string name, Reactor *parent, InputTuple inputs)
        : ReactionBase (name, parent), input_triggers(std::move(inputs)) {}
    ~ReactionInput() {}

    template <typename... Dependencies>
    ReactionDependency<InputTuple, std::tuple<Dependencies...>> &dependencies(Dependencies&&... deps)
    {
        auto deps_tuple = std::make_tuple(deps...);
        auto ReactionDependenciesRef = std::make_shared<ReactionDependency<InputTuple, std::tuple<Dependencies...>>> (name, reactor, std::move(input_triggers), std::move(deps_tuple));
        next = ReactionDependenciesRef;
        return *ReactionDependenciesRef;
    }
};

class ReactionName: public ReactionBase {
public:
    explicit ReactionName(std::string name, Reactor *parent)
        : ReactionBase (name, parent) {}
    ~ReactionName() = default;

    template <typename... Inputs>
    ReactionInput<std::tuple<Inputs...>> &triggers(Inputs&&... inputs)
    {
        auto input_tuple = std::make_tuple(inputs...);
        auto ReactionInputRef = std::make_shared<ReactionInput<std::tuple<Inputs...>>> (name, reactor, std::move(input_tuple));
        next = ReactionInputRef;
        return *ReactionInputRef;
    }

    // template <typename ReactorType, typename ParameterType>
    // friend class ReactionChamber;
};

template <typename Fn, typename InputTuple, typename DependencyTuple, typename OutputTuple>
class Reaction: public ReactionBase
{
private:
    InputTuple input_triggers;
    DependencyTuple dependencies;
    OutputTuple output_triggers;
    Fn user_function;
    std::unique_ptr<reactor::Reaction> reaction;

    template <typename Reaction, typename Trigger>
    void set_input_trigger(Reaction &reaction, Trigger &&trigger)
    {
        if constexpr (is_specialization_v<std::remove_pointer_t<std::decay_t<Trigger>>, MultiportInput>)
        {
            for (auto& port : *trigger) {
                reaction.declare_trigger(&port);
            }
        }
        else if constexpr (is_specialization_v<std::remove_pointer_t<std::decay_t<Trigger>>, MultiportOutput>)
        {
            for (auto& port : *trigger) {
                reaction.declare_trigger(&port);
            }
        }
        else {
            reaction.declare_trigger(trigger);
        }
    }

    template <typename Reaction, typename... Triggers>
    void set_input_triggers(std::unique_ptr<Reaction> &reaction, const std::tuple<Triggers...> &inputs)
    {
        std::apply([this, &reaction](auto &&...input)
            {
                (void)this;
                (..., set_input_trigger(*reaction, std::forward<decltype(input)>(input)));
            },
            inputs);
    }

    template <typename Reaction, typename Trigger>
    void set_dependency(Reaction &reaction, Trigger &&trigger)
    {
        if constexpr (is_specialization_v<std::remove_pointer_t<std::decay_t<Trigger>>, MultiportInput>)
        {
            for (auto& port : *trigger) {
                reaction.declare_dependency(&port);
            }
        }
        else {
            reaction.declare_dependency(trigger);
        }
    }

    template <typename Reaction, typename... Dependencies>
    void set_dependencies(std::unique_ptr<Reaction> &reaction, const std::tuple<Dependencies...> &deps)
    {
        std::apply([this, &reaction](auto &&...dep)
            {
                (void)this;
                (..., set_dependency(*reaction, std::forward<decltype(dep)>(dep)));
            },
            deps);
    }

    template <typename Reaction, typename Trigger>
    void set_output_trigger(Reaction &reaction, Trigger &&trigger)
    {
        if constexpr (is_specialization_v<std::remove_pointer_t<std::decay_t<Trigger>>, Output>)
        {
            reaction.declare_antidependency(trigger);
        } else if constexpr (is_specialization_v<std::remove_pointer_t<std::decay_t<Trigger>>, Input>)
        {
            reaction.declare_antidependency(trigger);
        }
        else if constexpr (is_specialization_v<std::remove_pointer_t<std::decay_t<Trigger>>, reactor::LogicalAction>)
        {
            reaction.declare_schedulable_action(trigger);
        }
        else if constexpr (is_specialization_v<std::remove_pointer_t<std::decay_t<Trigger>>, MultiportOutput>)
        {
            for (auto& port : *trigger) {
                reaction.declare_antidependency(&port);
            }
        }
        else
        {
            static_assert(templated_false<Trigger>, "Unsupported trigger type");
        }
    }

    template <typename Reaction, typename... Triggers>
    void set_output_triggers(std::unique_ptr<Reaction> &reaction, const std::tuple<Triggers...> &outputs)
    {
        std::apply([this, &reaction](auto &&...output)
            {
                (void)this;
                (..., set_output_trigger(*reaction, std::forward<decltype(output)>(output)));
            },
            outputs);
    }

    template <typename Reaction>
    void set_output_triggers(std::unique_ptr<Reaction>& reaction, const std::tuple<>& outputs) {}

public:
    Reaction(std::string name, Reactor *parent, InputTuple inputs, DependencyTuple deps, OutputTuple outputs, Fn func)
        : ReactionBase(name, parent), input_triggers(std::move(inputs)), dependencies(std::move(deps)), output_triggers(std::move(outputs)), user_function(std::forward<Fn>(func)) { /* std::cout << "Creating Reaction\n"; */ }
    ~Reaction() {}

    void execute () {
        int priority = reactor->get_priority();
        reactor->add_to_reaction_map(name, shared_from_this());
        reactor->validate_reaction (user_function, input_triggers, dependencies, output_triggers);

        auto reactor_func = [func = std::move(user_function), this]()
        {
            (void)this;
            auto apply_to_dereferenced = [](auto&& func, auto&& tuple) {
                return std::apply(
                    [&](auto*... ptrs) {
                        return std::invoke(std::forward<decltype(func)>(func), (*ptrs)...);
                    },
                    std::forward<decltype(tuple)>(tuple));
            };

            apply_to_dereferenced(func, std::tuple_cat(this->input_triggers, this->dependencies, this->output_triggers));
        };

        reaction = std::make_unique<reactor::Reaction>(name, priority, reactor, reactor_func);

        set_input_triggers(reaction, input_triggers);
        set_dependencies(reaction, dependencies);
        set_output_triggers(reaction, output_triggers);
    }

    template <typename Dfn>
    void deadline(reactor::Duration deadline_period, Dfn fn)
    {
        reactor->validate_reaction (fn, input_triggers, dependencies, output_triggers);

        auto deadline_func = [func = std::move(fn), this]()
        {
            (void)this;
            auto apply_to_dereferenced = [](auto&& func, auto&& tuple) {
                return std::apply(
                    [&](auto*... ptrs) {
                        return std::invoke(std::forward<decltype(func)>(func), (*ptrs)...);
                    },
                    std::forward<decltype(tuple)>(tuple));
            };

            apply_to_dereferenced(func, std::tuple_cat(this->input_triggers, this->dependencies, this->output_triggers));
        };

        reaction->set_deadline(deadline_period, deadline_func);
    }
};

template <typename ReactorType>
class ReactionChamberParameterless : public ReactionBase {
    ReactorType *reactor_;
protected:
    const size_t &bank_index = reactor_->bank_index;
public:
    ReactionChamberParameterless(Reactor *owner)
        : ReactionBase("reaction-internals-parameterless", owner), reactor_((ReactorType*) owner) {
        reactor_->add_reaction_internals(this);
    }

    ReactionName &reaction (const std::string name) {
        auto ReactionNameRef = std::make_shared<ReactionName>(name, reactor);
        next = ReactionNameRef;
        return *ReactionNameRef;
    }

    virtual void add_reactions(ReactorType *reactor) = 0;
    virtual void assemble() override {
        add_reactions(reactor_);
    }

    auto fqn() const noexcept -> const std::string& { return reactor_->fqn(); }
    auto get_elapsed_logical_time() const noexcept -> Duration { return reactor_->get_elapsed_logical_time(); }
    auto get_microstep() const noexcept -> reactor::mstep_t { return reactor_->get_microstep(); }
    auto get_elapsed_physical_time() const noexcept -> Duration { return reactor_->get_elapsed_physical_time(); }
    auto get_physical_time() noexcept -> reactor::TimePoint { return reactor_->get_physical_time(); }
    auto get_logical_time() const noexcept -> reactor::TimePoint { return reactor_->get_logical_time(); }
    auto get_tag() const noexcept -> reactor::Tag { return reactor_->get_tag(); }
    void request_stop() { reactor_->environment()->sync_shutdown(); }
};

template <typename ReactorType, typename ParameterType>
class ReactionChamber : public ReactionBase {
    ReactorType *reactor_;

protected:
    const ParameterType &parameters;
    const size_t &bank_index = reactor_->bank_index;
public:
    ReactionChamber(Reactor *owner, ParameterType &param)
        : ReactionBase("reaction-internals", owner), reactor_((ReactorType*) owner), parameters(param) {
        reactor_->add_reaction_internals(this);
    }

    ReactionName &reaction (const std::string name) {
        auto ReactionNameRef = std::make_shared<ReactionName>(name, reactor);
        next = ReactionNameRef;
        return *ReactionNameRef;
    }

    virtual void add_reactions(ReactorType *reactor) = 0;
    virtual void assemble() override {
        add_reactions(reactor_);
    }

    auto fqn() const noexcept -> const std::string& { return reactor_->fqn(); }
    auto get_elapsed_logical_time() const noexcept -> Duration { return reactor_->get_elapsed_logical_time(); }
    auto get_microstep() const noexcept -> reactor::mstep_t { return reactor_->get_microstep(); }
    auto get_elapsed_physical_time() const noexcept -> Duration { return reactor_->get_elapsed_physical_time(); }
    auto get_physical_time() noexcept -> reactor::TimePoint { return reactor_->get_physical_time(); }
    auto get_logical_time() const noexcept -> reactor::TimePoint { return reactor_->get_logical_time(); }
    auto get_tag() const noexcept -> reactor::Tag { return reactor_->get_tag(); }
    void request_stop() { reactor_->environment()->sync_shutdown(); }
};

#define REACTION_SCOPE_START(ReactorType, ParamType) \
class Internals : public ReactionChamber<ReactorType, ParamType> { \
public: \
    Internals(Reactor *reactor, ParamType &params) \
        : ReactionChamber<ReactorType, ParamType>(reactor, params) {} \
private:

#define REACTION_SCOPE_END(reactor, param) \
}; \
Internals reaction_internals{reactor, param};

#define REACTION_SCOPE_START_NO_PARAMS(ReactorType) \
class Internals : public ReactionChamberParameterless<ReactorType> { \
public: \
    Internals(Reactor *reactor) \
        : ReactionChamberParameterless<ReactorType>(reactor) {} \
private:

#define REACTION_SCOPE_END_NO_PARAMS(reactor) \
}; \
Internals reaction_internals{reactor};

#define REACTION_SCOPE(ReactorType) ReactorType::Internals

} // namespace sdk