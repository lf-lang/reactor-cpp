#pragma once

#include <reactor-sdk/reactor-sdk.hh>

#include "Source/SourceReactor.hh"
#include "Sink/SinkReactor.hh"

using namespace sdk;

class MainReactor: public Reactor {
public:
    struct Parameters : public SystemParameter<string> {
        REACTOR_PARAMETER (string, alias, "Alternate name", "another", "another", "Src-Sink-Fanout-Example");

        Parameters(Reactor *container)
            :   SystemParameter<string>(container) {
            register_parameters (alias);
        }
  };

private:
    Parameters parameters{this};

    std::unique_ptr<SourceReactor> src;
    std::unique_ptr<SinkReactor> snk;

public:
    MainReactor(const std::string &name, Environment *env)
    : Reactor(name, env) {}
    MainReactor(const std::string &name, Reactor *container)
    : Reactor(name, container) {}
  
    void construction() override;
    void assembling() override;
};

        
