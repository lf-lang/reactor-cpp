#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SinkReactor : public Reactor {
public:
    struct Parameters : public SystemParameter<string> {
        ParameterMetadata<string> name = ParameterMetadata<string> {
            .name = "Name",
            .description = "Alternate name",
            .min_value = "Sink",
            .max_value = "Sink",
            .value = "Sink"
        };

        Parameters(Reactor *container)
            :   SystemParameter<string>(container) {
            register_parameters (name);
        }
    };

    SinkReactor(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    SinkReactor(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    Parameters parameters{this};

    Input<int> req{"req", this};
    Output<int> rsp{"rsp", this};
    
    void construction() override;
    void assembling() override;

    void startup_reaction (Startup &startup);
    void process_request (Input<int>& req, Output<int>& rsp);
};
