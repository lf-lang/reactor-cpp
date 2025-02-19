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

        Parameters(Reactor *container)
            :   SystemParameter<int>(container) {
            register_parameters (iterations);
        }
    };
private:
    Parameters parameters{this};

    std::string name = "Source";
    int itr = 0;
public:                                                         
    SourceReactor(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    SourceReactor(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    LogicalAction<int> sch{"sch", this};
    Input<int> rsp{"rsp", this};
    Output<int> req{"req", this};

    void construction() override;
    void assembling() override;
};