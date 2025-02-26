#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SourceReactor : public Reactor {
public:
    struct Parameters : public SystemParameter<int> {
        REACTOR_PARAMETER (int, iterations, "Number of iterations", 1, 100, 10);

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