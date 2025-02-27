#pragma once

#include "reactor-cpp/reactor-cpp.hh"
#include "BaseTrigger.hh"
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



template <typename InputTuple, typename DependencyTuple, typename OutputTuple>
class ReactionOutput: public BaseTrigger
{
private:
    InputTuple input_triggers;
    DependencyTuple dependencies;
    OutputTuple output_triggers;

public:
    explicit ReactionOutput(std::string name, Reactor *parent, InputTuple inputs, DependencyTuple deps, OutputTuple outputs)
        : BaseTrigger (name, parent), input_triggers(std::move(inputs)), dependencies(std::move(deps)), output_triggers(std::move(outputs)) {}
    ~ReactionOutput() {}

    template <typename Fn>
    Reaction<Fn, InputTuple, DependencyTuple, OutputTuple> &function(Fn func)
    {
        auto ReactionRef = std::make_shared<Reaction<Fn, InputTuple, DependencyTuple, OutputTuple>> (name, reactor, std::move(input_triggers), std::move(dependencies), std::move(output_triggers), std::forward<Fn>(func));
        ReactionRef->execute();
        return *ReactionRef;
    }
};

template <typename InputTuple, typename DependencyTuple>
class ReactionDependency: public BaseTrigger
{
private:
    InputTuple input_triggers;
    DependencyTuple dependencies;

public:
    explicit ReactionDependency(std::string name, Reactor *parent, InputTuple inputs, DependencyTuple deps)
        : BaseTrigger (name, parent), input_triggers(inputs), dependencies(std::move(deps)) {}
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
class ReactionInput: public BaseTrigger
{
private:
    InputTuple input_triggers;

public:
    explicit ReactionInput(std::string name, Reactor *parent, InputTuple inputs)
        : BaseTrigger (name, parent), input_triggers(std::move(inputs)) {}
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

template <typename Fn, typename InputTuple, typename DependencyTuple, typename OutputTuple>
class Reaction: public BaseTrigger
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
            static_assert(false, "Unsupported trigger type");
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

public:
    Reaction(std::string name, Reactor *parent, InputTuple inputs, DependencyTuple deps, OutputTuple outputs, Fn func)
        : BaseTrigger(name, parent), input_triggers(std::move(inputs)), dependencies(std::move(deps)), output_triggers(std::move(outputs)), user_function(std::forward<Fn>(func)) { /* std::cout << "Creating Reaction\n"; */ }
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

} // namespace sdk