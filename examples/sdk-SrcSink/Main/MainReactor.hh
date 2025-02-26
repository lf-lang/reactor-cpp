#pragma once

#include <reactor-sdk/reactor-sdk.hh>

#include "Source/SourceReactor.hh"
#include "Sink/SinkReactor.hh"

using namespace sdk;

class MainReactor: public Reactor {
public:
    struct Parameters : public SystemParameter<string, int> {
        REACTOR_PARAMETER(string, alias, "Alternate name", "another", "another", "Src-Sink-Example");
        REACTOR_PARAMETER(int, n_sinks, "Sink reactors bank width", 1, 10, 1);

        Parameters(Reactor *container)
            :   SystemParameter<string, int>(container) {
            register_parameters (alias, n_sinks);
        }
  };

private:
    Parameters parameters{this};

    std::unique_ptr<SourceReactor> src;
    ReactorBank<SinkReactor> snk{"Sink", this};

public:
    MainReactor(const std::string &name, Environment *env)
    : Reactor(name, env) {}
    MainReactor(const std::string &name, Reactor *container)
    : Reactor(name, container) {}
  
    void construction() override;
    void assembling() override;
};

        
