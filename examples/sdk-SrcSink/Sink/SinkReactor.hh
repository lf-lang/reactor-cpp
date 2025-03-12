#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SinkReactor : public Reactor {
public:

    struct Parameters {
        string name = "Sink";
    };
private:
    struct PublishParameters : public SystemParameters<Parameters, string> {
        REACTOR_PARAMETER(string, name, "Alternate name", "Sink", "Sink", defaults.name);

        PublishParameters(Reactor *container, Parameters &&param)
            :   SystemParameters<Parameters, string>(container, std::forward<Parameters>(param)) {
            register_parameters (name);
        }
    };
    PublishParameters parameters;

    REACTION_SCOPE_START(SinkReactor, PublishParameters)
        void add_reactions(SinkReactor *reactor);
        
        void startup_reaction (Startup &startup);
        void process_request (Input<int>& req, Output<int>& rsp);
    REACTION_SCOPE_END(this, parameters)

public:
    SinkReactor(const std::string &name, Environment *env)
    : Reactor(name, env), parameters{this, Parameters{}} {}
    SinkReactor(const std::string &name, Reactor *container)
    : Reactor(name, container), parameters{this, Parameters{}} {}

    SinkReactor(const std::string &name, Environment *env, Parameters && param)
    : Reactor(name, env), parameters{this, std::forward<Parameters>(param)} {}
    SinkReactor(const std::string &name, Reactor *container, Parameters && param)
    : Reactor(name, container), parameters{this, std::forward<Parameters>(param)} {}

    Input<int> req{"req", this};
    Output<int> rsp{"rsp", this};
    
    void construction() override;
    void wiring() override;
};
