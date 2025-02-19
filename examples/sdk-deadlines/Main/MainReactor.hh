#pragma once

#include <reactor-sdk/reactor-sdk.hh>

#include "Node/NodeReactor.hh"

using namespace sdk;

class MainReactor: public Reactor {
public:
    struct Parameters : public SystemParameter<int> {
        ParameterMetadata<int> n_fast = ParameterMetadata<int> {
            .name = "n_fast",
            .description = "Number of fast nodes",
            .min_value = 1,
            .max_value = 10,
            .value = 2
        };

        Parameters(Reactor *container)
            :   SystemParameter<int>(container) {
            register_parameters (n_fast);
        }
    };
private:
    Parameters parameters{this};
    std::unique_ptr<NodeReactor> slow;
    ReactorBank<NodeReactor> fast;

public:
    MainReactor(const std::string &name, Environment *env)
    : Reactor(name, env) {}
    MainReactor(const std::string &name, Reactor *container)
    : Reactor(name, container) {}
  
    void construction() override;
    void assembling() override;
};

        
