#pragma once

#include <reactor-sdk/reactor-sdk.hh>

#include "Source/SourceReactor.hh"
#include "Sink/SinkReactor.hh"

using namespace sdk;

class MainReactor: public Reactor {
public:
    struct Parameters {
        string alias = "Src-Sink-Example";
        int n_sinks = 2;
    };
private:
    struct PublishParameters : public SystemParameters<Parameters, string, int> {
        REACTOR_PARAMETER(string, alias, "Alternate name", "another", "another", defaults.alias);
        REACTOR_PARAMETER(int, n_sinks, "Sink reactors bank width", 1, 10, defaults.n_sinks);

        PublishParameters(Reactor *container, Parameters &&param)
            :   SystemParameters<Parameters, string, int>(container, std::forward<Parameters>(param)) {
            register_parameters (alias, n_sinks);
        }
    };
    PublishParameters parameters;

    REACTION_SCOPE_START(MainReactor, PublishParameters)
        void add_reactions(MainReactor *reactor);
    REACTION_SCOPE_END(this, parameters)

    std::unique_ptr<SourceReactor> src;
    ReactorBank<SinkReactor> snk{"Sink", this};

public:
    MainReactor(const std::string &name, Environment *env)
    : Reactor(name, env), parameters{this, Parameters{}} {}
    MainReactor(const std::string &name, Reactor *container)
    : Reactor(name, container), parameters{this, Parameters{}} {}

    MainReactor(const std::string &name, Environment *env, Parameters && param)
    : Reactor(name, env), parameters{this, std::forward<Parameters>(param)} {}
    MainReactor(const std::string &name, Reactor *container, Parameters && param)
    : Reactor(name, container), parameters{this, std::forward<Parameters>(param)} {}
  
    void construction() override;
    void wiring() override;
};