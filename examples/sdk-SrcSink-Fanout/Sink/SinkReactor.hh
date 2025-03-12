#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SinkReactor : public Reactor {
public:

    struct Parameters {
        string name = "Sink";
        int n_ports = 1;
    };
private:
    struct PublishParameters : public SystemParameters<Parameters, string, int> {
        REACTOR_PARAMETER(string, name, "Alternate name", "Sink", "Sink", defaults.name);
        REACTOR_PARAMETER (int, n_ports, "Size of multiports", 1, 10, defaults.n_ports);

        PublishParameters(Reactor *container, Parameters &&param)
            :   SystemParameters<Parameters, string, int>(container, std::forward<Parameters>(param)) {
            register_parameters (name, n_ports);
        }
    };
    PublishParameters parameters;

    REACTION_SCOPE_START(SinkReactor, PublishParameters)
        void add_reactions(SinkReactor *reactor);
        
        void startup_reaction (Startup &startup);
        void process_request (MultiportInput<int>& req, MultiportOutput<int>& rsp);
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

    MultiportInput<int> req{"req", this};
    MultiportOutput<int> rsp{"rsp", this};
    
    void construction() override;
    void wiring() override;
};
