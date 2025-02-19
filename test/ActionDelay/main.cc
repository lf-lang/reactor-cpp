
#include <reactor-sdk/reactor-sdk.hh>

using namespace std;
using namespace sdk;

class GeneratedDelay : public Reactor {
    int y_state = 0;
    LogicalAction<void> act{"act", this, 100ms};
public:
    GeneratedDelay(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    GeneratedDelay(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    Input<int> y_in{"y_in", this};
    Output<int> y_out{"y_out", this};

    void construction() {}

    void assembling() {
        reaction ("reaction_1").
            triggers(&y_in).
            effects(&act).
            function (
                [&](Input<int> &y_in, LogicalAction<void> &act) {
                    y_state = *y_in.get();
                    act.schedule();
                }
            );
        
        reaction ("reaction_2").
            triggers(&act).
            effects(&y_out).
            function (
                [&](LogicalAction<void> &act, Output<int> &y_out) {
                    y_out.set(y_state);
                }
            );
    }
};

class Source : public Reactor {
public:
    Source(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    Source(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    Output<int> out{"out", this};

    void construction() {}

    void assembling() {
        reaction ("reaction_1").
            triggers(&startup).
            effects(&out).
            function (
                [&](Startup &startup, Output<int> &out) {
                    out.set(1);
                }
            );
    }
};

class Sink : public Reactor {
public:
    Sink(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    Sink(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    Input<int> in{"in", this};

    void construction() {}

    void assembling() {
        reaction ("reaction_1").
            triggers(&in).
            effects().
            function (
                [&](Input<int> &in) {
                    auto elapsed_logical = get_elapsed_logical_time();
                    auto logical = get_logical_time();
                    auto physical = get_physical_time();
                    std::cout << "logical time: " << logical << '\n';
                    std::cout << "physical time: " << physical << '\n';
                    std::cout << "elapsed logical time: " << elapsed_logical << '\n';
                    if (elapsed_logical != 100ms) {
                    std::cerr << "ERROR: Expected 100 msecs but got " << elapsed_logical << '\n';
                    exit(1);
                    } else {
                    std::cout << "SUCCESS. Elapsed logical time is 100 msec.\n";
                    }
                }
            );
    }
};

class ActionDelay : public Reactor {
    std::unique_ptr<Source> source;
    std::unique_ptr<Sink> sink;
    std::unique_ptr<GeneratedDelay> g;
public:
    ActionDelay(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    ActionDelay(const std::string &name, Reactor *container)
        : Reactor(name, container) {}
    
    void construction() {
        source = std::make_unique<Source>("Source", this);
        sink = std::make_unique<Sink>("Sink", this);
        g = std::make_unique<GeneratedDelay>("GeneratedDelay", this);
    }

    void assembling() {
        source->out --> g->y_in;
        g->y_out --> sink->in;
    }
};

int main(int argc, char **argv) {
    unsigned workers = 1;
    bool fast{false};
    reactor::Duration timeout = reactor::Duration::max();

    bool visualize = false;

    if (argc > 1) {
        string v_str = argv[1];
        visualize =  (v_str == "true") ? true : false;
    }

    std::cout << "parameters - workers:" << workers << " fast:" << (fast ? "True" : "False") << " timeout:" << timeout << " visualize:" << visualize << std::endl;

    Environment sim {nullptr, workers, fast, timeout, visualize};
    auto action_delay = new ActionDelay("ActionDelay", &sim);

    sim.run();
    return 0;
}
