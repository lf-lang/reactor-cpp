#pragma once

#include <reactor-sdk/reactor-sdk.hh>

#include "Source/SourceReactor.hh"
#include "Sink/SinkReactor.hh"

using namespace sdk;

class MainReactor: public Reactor {
public:

    struct DefaultParameters {
        string alias = "Src-Sink-Example";
        int n_sinks = 2;
    };

    struct Parameters : public SystemParameterWithDefault<DefaultParameters, string, int> {
        REACTOR_PARAMETER(string, alias, "Alternate name", "another", "another", defaults.alias);
        REACTOR_PARAMETER(int, n_sinks, "Sink reactors bank width", 1, 10, defaults.n_sinks);

        // Parameters(Reactor *container)
        //     :   SystemParameter<string, int>(container) {
        //     register_parameters (alias, n_sinks);
        // }

        Parameters(Reactor *container, DefaultParameters &&param)
            :   SystemParameterWithDefault<DefaultParameters, string, int>(container, std::forward<DefaultParameters>(param)) {
            register_parameters (alias, n_sinks);
        }
    };

private:
    Parameters parameters;

    std::unique_ptr<SourceReactor> src;
    ReactorBank<SinkReactor> snk{"Sink", this};

public:
    MainReactor(const std::string &name, Environment *env)
    : Reactor(name, env), parameters{this, DefaultParameters{}} {}
    MainReactor(const std::string &name, Reactor *container)
    : Reactor(name, container), parameters{this, DefaultParameters{}} {}

    MainReactor(const std::string &name, Environment *env, DefaultParameters && param)
    : Reactor(name, env), parameters{this, std::forward<DefaultParameters>(param)} {}
    MainReactor(const std::string &name, Reactor *container, DefaultParameters && param)
    : Reactor(name, container), parameters{this, std::forward<DefaultParameters>(param)} {}
  
    void construction() override;
    void assembling() override;
};

        
