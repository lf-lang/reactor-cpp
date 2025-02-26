#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SinkReactor : public Reactor {
public:
    struct Parameters : public SystemParameter<string, int> {
        REACTOR_PARAMETER (string, name, "Alternate name", "Sink", "Sink", "Sink");
        REACTOR_PARAMETER (int, n_ports, "Size of multiports", 1, 10, 1);

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
