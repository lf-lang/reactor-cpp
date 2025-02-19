#pragma once

#include <reactor-sdk/reactor-sdk.hh>

#include "Source/SourceReactor.hh"
#include "Sink/SinkReactor.hh"

using namespace sdk;

class MainReactor: public Reactor {
public:
    struct Parameters : public SystemParameter<string, int> {
        ParameterMetadata<string> alias = ParameterMetadata<string> {
            .name = "alias",
            .description = "Alternate name",
            .min_value = "another",
            .max_value = "another",
            .value = "another"
        };

        ParameterMetadata<int> n_sinks = ParameterMetadata<int> {
            .name = "n_sinks",
            .description = "Sink reactors bank width",
            .min_value = 1,
            .max_value = 10,
            .value = 1
        };

        ParameterMetadata<int> log_level = ParameterMetadata<int> {
            .name = "log_level",
            .description = "Log level",
            .min_value = 0,
            .max_value = 1,
            .value = 1
        };

        Parameters(Reactor *container)
            :   SystemParameter<string, int>(container) {
            register_parameters (alias, n_sinks, log_level);
        }
  };

private:
    Parameters parameters{this};

    std::unique_ptr<SourceReactor> src;
    ReactorBank<SinkReactor> snk;

public:
    MainReactor(const std::string &name, Environment *env)
    : Reactor(name, env) {}
    MainReactor(const std::string &name, Reactor *container)
    : Reactor(name, container) {}
  
    void construction() override;
    void assembling() override;
};

        
