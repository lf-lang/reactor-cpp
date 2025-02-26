#pragma once

#include <reactor-sdk/reactor-sdk.hh>

#include "Node/NodeReactor.hh"

using namespace sdk;

class MainReactor: public Reactor {
public:
    struct Parameters : public SystemParameter<int> {
        REACTOR_PARAMETER(int, n_fast, "Number of fast nodes", 1, 10, 2);

        Parameters(Reactor *container)
            :   SystemParameter<int>(container) {
            register_parameters (n_fast);
        }
    };
private:
    Parameters parameters{this};
    std::unique_ptr<NodeReactor> slow;
    ReactorBank<NodeReactor> fast{"fast", this};

public:
    MainReactor(const std::string &name, Environment *env)
    : Reactor(name, env) {}
    MainReactor(const std::string &name, Reactor *container)
    : Reactor(name, container) {}
  
    void construction() override;
    void assembling() override;
};

        
