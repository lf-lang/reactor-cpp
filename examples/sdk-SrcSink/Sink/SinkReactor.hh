#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SinkReactor : public Reactor {
public:

    struct DefaultParameters {
        string name = "Sink";
    };
    struct Parameters : public SystemParameterWithDefault<DefaultParameters, string> {
        REACTOR_PARAMETER(string, name, "Alternate name", "Sink", "Sink", defaults.name);

        Parameters(Reactor *container, DefaultParameters &&param)
            :   SystemParameterWithDefault<DefaultParameters, string>(container, std::forward<DefaultParameters>(param)) {
            register_parameters (name);
        }
    };

    SinkReactor(const std::string &name, Environment *env)
    : Reactor(name, env), parameters{this, DefaultParameters{}} {}
    SinkReactor(const std::string &name, Reactor *container)
    : Reactor(name, container), parameters{this, DefaultParameters{}} {}

    SinkReactor(const std::string &name, Environment *env, DefaultParameters && param)
    : Reactor(name, env), parameters{this, std::forward<DefaultParameters>(param)} {}
    SinkReactor(const std::string &name, Reactor *container, DefaultParameters && param)
    : Reactor(name, container), parameters{this, std::forward<DefaultParameters>(param)} {}

    Parameters parameters;

    Input<int> req{"req", this};
    Output<int> rsp{"rsp", this};
    
    void construction() override;
    void assembling() override;

    void startup_reaction (Startup &startup);
    void process_request (Input<int>& req, Output<int>& rsp);
};
