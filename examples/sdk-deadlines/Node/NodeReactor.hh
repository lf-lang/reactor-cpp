#pragma once

#include <reactor-sdk/reactor-sdk.hh>

using namespace sdk;

class NodeReactor: public Reactor {
public:
    struct Parameters : public SystemParameter<Duration> {
        ParameterMetadata<Duration> period = ParameterMetadata<Duration> {
            .name = "period",
            .description = "Schedule and deadline period",
            .min_value = 10ms,
            .max_value = 10s,
            .value = 500ms
        };

        ParameterMetadata<Duration> duration = ParameterMetadata<Duration> {
            .name = "duration",
            .description = "Sleep duration",
            .min_value = 5ms,
            .max_value = 5s,
            .value = 10ms
        };

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

        
