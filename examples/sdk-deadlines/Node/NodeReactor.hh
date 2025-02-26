#pragma once

#include <reactor-sdk/reactor-sdk.hh>

using namespace sdk;

class NodeReactor: public Reactor {
public:
    struct Parameters : public SystemParameter<Duration> {
        REACTOR_PARAMETER(Duration, period, "Schedule and deadline period", 10ms, 10s, 500ms);
        REACTOR_PARAMETER(Duration, duration, "Sleep duration", 5ms, 5s, 10ms);

        Parameters(Reactor *container)
            :   SystemParameter<Duration>(container) {
            register_parameters (period, duration);
        }
  };

private:
    Parameters parameters{this};
    LogicalAction<void> a{"a", this};

public:
    NodeReactor(const std::string &name, Environment *env)
    : Reactor(name, env) {}
    NodeReactor(const std::string &name, Reactor *container)
    : Reactor(name, container) {}
  
    void construction() override;
    void assembling() override;
};

        
