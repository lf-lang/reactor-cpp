#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SinkReactor : public Reactor {
public:
    struct Parameters : public SystemParameter<string, int> {
        ParameterMetadata<string> name = ParameterMetadata<string> {
            .name = "Name",
            .description = "Alternate name",
            .min_value = "Sink",
            .max_value = "Sink",
            .value = "Sink"
        };

        ParameterMetadata<int> n_ports = ParameterMetadata<int> {
            .name = "n_ports",
            .description = "Size of multiports",
            .min_value = 1,
            .max_value = 10,
            .value = 1
        };

        Parameters(Reactor *container)
            :   SystemParameter<string, int>(container) {
            register_parameters (name, n_ports);
        }
    };

    SinkReactor(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    SinkReactor(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    Parameters parameters{this};

    MultiportInput<int> req{"req", this};
    MultiportOutput<int> rsp{"rsp", this};
    
    void construction() override;
    void assembling() override;

    void startup_reaction (Startup &startup);
    void process_request (MultiportInput<int>& req, MultiportOutput<int>& rsp);
};
