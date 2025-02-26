
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
    cxxopts::Options options("ActionDelay", "Multiport source connecting to banked sink reactors");

    unsigned workers = std::thread::hardware_concurrency();
    bool fast{false};
    reactor::Duration timeout = reactor::Duration::max();
    bool cfg_gen{false};

    // the timeout variable needs to be tested beyond fitting the Duration-type 
    options
    .set_width(120)
    .add_options()
        ("w,workers", "the number of worker threads used by the scheduler", cxxopts::value<unsigned>(workers)->default_value(std::to_string(workers)), "'unsigned'")
        ("o,timeout", "Time after which the execution is aborted.", cxxopts::value<reactor::Duration>(timeout)->default_value(time_to_string(timeout)), "'FLOAT UNIT'")
        ("f,fast", "Allow logical time to run faster than physical time.", cxxopts::value<bool>(fast)->default_value("false"))
        ("c,config-gen", "Generate configuration files for the topology.", cxxopts::value<bool>(cfg_gen)->default_value("false"))
        ("help", "Print help");

    cxxopts::ParseResult result{};
    bool parse_error{false};
    try {
    result = options.parse(argc, argv);
    } catch (const cxxopts::OptionException& e) {
    reactor::log::Error() << e.what();
    parse_error = true;
    }

    // if parameter --help was used or there was a parse error, print help
    if (parse_error || result.count("help"))
    {
        std::cout << options.help({""});
        return parse_error ? -1 : 0;
    }

    std::cout << "parameters - workers:" << workers << " fast:" << (fast ? "True" : "False") << " timeout:" << timeout << " cfg_gen:" << (cfg_gen ? "True" : "False") << std::endl;

    Environment sim {nullptr, workers, fast, timeout, cfg_gen};
    auto action_delay = new ActionDelay("ActionDelay", &sim);

    sim.run();
    return 0;
}
