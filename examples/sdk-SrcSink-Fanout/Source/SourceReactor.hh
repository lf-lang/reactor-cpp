#pragma once

#include <reactor-sdk/reactor-sdk.hh>
using namespace std;
using namespace sdk;

class SourceReactor : public Reactor {
public:
    struct Parameters : public SystemParametersStandalone<int> {
        REACTOR_PARAMETER(int, iterations, "Number of iterations", 1, 100, 10);

        Parameters(Reactor *container)
            :   SystemParametersStandalone<int>(container) {
            register_parameters (iterations);
        }
    };
private:
    LogicalAction<int> sch{"sch", this};
    
    Parameters parameters{this};

    REACTION_SCOPE_START(SourceReactor, Parameters)
        std::string name = "Source";
        int itr = 0;

        void add_reactions(SourceReactor *reactor);
    REACTION_SCOPE_END(this, parameters)

public:                                                         
    SourceReactor(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    SourceReactor(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    Input<int> rsp{"rsp", this};
    Output<int> req{"req", this};

    void construction() override;
    void wiring() override;
};