#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SourceReactor : public Reactor {
public:
    struct Parameters : public SystemParameter<int> {

        ParameterMetadata<int> iterations = ParameterMetadata<int> {
            .name = "iterations",
            .description = "Number of iterations",
            .min_value = 1,
            .max_value = 100,
            .value = 10
        };

        ParameterMetadata<int> n_ports = ParameterMetadata<int> {
            .name = "n_ports",
            .description = "Size of multiports",
            .min_value = 1,
            .max_value = 10,
            .value = 1
        };

        Parameters(Reactor *container)
            :   SystemParameter<int>(container) {
            register_parameters (iterations, n_ports);
        }
    };
private:
    Parameters parameters{this};

    std::string name = "Source";
    int itr = 0;
    int rsp_itr = 0;
public:                                                         
    SourceReactor(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    SourceReactor(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    LogicalAction<int> sch{"sch", this};
    MultiportInput<int> rsp{"rsp", this};
    MultiportOutput<int> req{"req", this};

    void construction() override;
    void assembling() override;
};