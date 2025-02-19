#pragma once

#include "reactor-cpp/reactor-cpp.hh"

namespace sdk
{

class Reactor;

template<typename T>
using LogicalAction = reactor::LogicalAction<T>;

using Startup = reactor::StartupTrigger;
using Shutdown = reactor::ShutdownTrigger;

using Duration = std::chrono::nanoseconds;

#define select_default(obj) &obj[0]

template <typename T>
struct inspect_function_args;

template <typename Ret, typename Class, typename... Args>
struct inspect_function_args<Ret(Class::*)(Args...)> {
    static constexpr size_t nargs = sizeof...(Args);
};

template <typename Func, typename Object>
auto bind_function(Object* obj, Func&& func) {
    constexpr size_t nargs = inspect_function_args<Func>::nargs;

    if constexpr (nargs == 0) {
        static_assert(nargs > 0, "Reactors must have one or more parameters");
        return nullptr;
    } else if constexpr (nargs == 1) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1);
    } else if constexpr (nargs == 2) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2);
    } else if constexpr (nargs == 3) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    } else if constexpr (nargs == 4) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    } else if constexpr (nargs == 5) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5);
    } else if constexpr (nargs == 6) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
    } else if constexpr (nargs == 7) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7);
    } else if constexpr (nargs == 8) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8);
    } else if constexpr (nargs == 9) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8, std::placeholders::_9);
    } else if constexpr (nargs == 10) {
        return std::bind(std::forward<Func>(func), obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8, std::placeholders::_9, std::placeholders::_10);
    } else {
        static_assert(nargs <= 10, "This needs to be extended as per requirement of more parameters");
        return nullptr;
    }

}

#define pass_function(func) \
    bind_function(this, &std::decay_t<decltype(*this)>::func)

class Timer : public reactor::Timer {
    std::string name;
    Reactor *reactor;
public:
    Timer(const std::string& name, Reactor* container)
        : reactor::Timer(name, (reactor::Reactor *) container), name (name), reactor (container){}

    void set_timer (Duration period = Duration::zero(), Duration offset = Duration::zero()) {
        period_ = period;
        offset_ = offset;
    }

    Timer(Timer&&) noexcept = default;
};


} // namespace sdk