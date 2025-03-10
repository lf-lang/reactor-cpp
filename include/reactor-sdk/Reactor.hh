#pragma once

#include "reactor-cpp/reactor-cpp.hh"
#include <string.h>
#include "ReactionBase.hh"
#include "SystemParameterBase.hh"
#include "Environment.hh"

namespace sdk
{

template <typename InputTuple>
class ReactionInput;

template <typename Fn, typename InputTuple, typename DependencyTuple, typename OutputTuple>
class Reaction;

template<typename T>
class Input;

template<typename T>
class Output;

template<typename T>
class MultiportOutput;

template<typename T>
class MultiportInput;

class Timer;

template <typename T>
struct trigger_value_type;

template <typename T>
struct trigger_value_type<reactor::LogicalAction<T> *>
{
    using type = reactor::LogicalAction<T>&;
};

template <typename T>
struct trigger_value_type<Input<T> *>
{
    using type = Input<T>&;
};

template <typename T>
struct trigger_value_type<MultiportInput<T> *>
{
    using type = MultiportInput<T>&;
};

template <typename T>
struct trigger_value_type<MultiportOutput<T> *>
{
    using type = MultiportOutput<T>&;
};

template <typename T>
struct trigger_value_type<Output<T> *>
{
    using type = Output<T>&;
};

template <>
struct trigger_value_type<reactor::StartupTrigger *>
{
    using type = reactor::StartupTrigger&;
};

template <>
struct trigger_value_type<reactor::ShutdownTrigger *>
{
    using type = reactor::ShutdownTrigger&;
};

template <>
struct trigger_value_type<Timer *>
{
    using type = Timer&;
};

class Reactor : public reactor::Reactor
{
protected:
    reactor::StartupTrigger startup{"startup", this};
    reactor::ShutdownTrigger shutdown{"shutdown", this};

private:
    size_t bank_index_ = 0;
    SystemParameterBase *p_param = nullptr;
    Environment *env{nullptr};
    Reactor *parent{nullptr};
    std::unordered_map<std::string, std::shared_ptr<ReactionBase>> reaction_map;
    ReactionBase *reaction_internals_;
    int priority = 1;
    std::set<Reactor*> child_reactors;
    std::string homog_name = "";

    void add_child(Reactor* reactor);
    void add_to_reaction_map (std::string &name, std::shared_ptr<ReactionBase> reaction);
    int get_priority() { return priority++;}

    template <typename Fn, typename... InputTriggers, typename... Dependencies, typename... OutputTriggers>
    void validate_reaction(Fn func, std::tuple<InputTriggers...> inputs, std::tuple<Dependencies...> deps, std::tuple<OutputTriggers...> outputs) {
        (void)func;
        (void)inputs;
        (void)deps;
        (void)outputs;
        static_assert(
            std::is_invocable_v<
                Fn,
                typename trigger_value_type<InputTriggers>::type...,
                typename trigger_value_type<Dependencies>::type...,
                typename trigger_value_type<OutputTriggers>::type...
                >,
                "Reaction function parameters must match the declared input and output types.");
    }

    void populate_params(std::set<std::string> &types, std::map<std::string, std::string> &homog_map_entries, std::map<std::string, std::string> &hetero_map_entries);

public:
    const size_t &bank_index = bank_index_;

    Reactor(const std::string &name, Environment *env);
    Reactor(const std::string &name, Reactor *container);

    void add_reaction_internals (ReactionBase* internals) {
        reaction_internals_ = internals;
    }

    static std::string BankName(const std::string& name);
    static std::string HomogName(const std::string& name);

    void set_param (SystemParameterBase *param) { p_param = param; }

    Environment *get_env() { return env; }

    auto homog_fqn() const noexcept -> const std::string& { return homog_name; }

    virtual void construction() = 0;
    virtual void wiring() = 0;
    void construct() override;
    void assemble() override;

    template <typename Fn, typename InputTuple, typename DependencyTuple, typename OutputTuple>
    friend class Reaction;

    template <typename ReactorType>
    friend class ReactorBank;

    friend class Environment;
};

} // namespace sdk