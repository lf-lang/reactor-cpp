
#include <reactor-sdk/reactor-sdk.hh>

using namespace std;
using namespace sdk;

class Src : public Reactor {
    REACTION_SCOPE_START_NO_PARAMS(Src)
        void add_reactions (Src *reactor) {
            reaction ("reaction_1").
                triggers(&reactor->startup).
                dependencies().
                effects(&reactor->out).
                function (
                    [this](Startup &startup, Output<unsigned> &out) {
                        out.set(42);
                    }
                );
        }
    REACTION_SCOPE_END_NO_PARAMS(this)
public:
    Output<unsigned> out{"out", this};

    Src(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    Src(const std::string &name, Reactor *container)
        : Reactor(name, container) {}
    
    void construction() {}
    void wiring() {}
};

class Sink : public Reactor {
    REACTION_SCOPE_START_NO_PARAMS(Sink)
        void add_reactions (Sink *reactor) {
            reaction ("reaction_1").
                triggers(&reactor->startup).
                dependencies(&reactor->in).
                effects().
                function (
                    [this](Startup &startup, Input<unsigned> &in) {
                        if (!in.is_present()) {
                            reactor::log::Error() << "Received no value";
                            exit(1);
                        }
                        if(*in.get() != 42) {
                            reactor::log::Error() << "Received an unexpected value";
                            exit(1);
                        }
                    }
                );

            reaction ("reaction_2").
                triggers(&reactor->shutdown).
                dependencies().
                effects().
                function (
                    [this](Shutdown &shutdown) {
                        reactor::log::Info() << "Success!";
                    }
                );
        }
    REACTION_SCOPE_END_NO_PARAMS(this)
public:
    Input<unsigned> in{"in", this};

    Sink(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    Sink(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    void construction() {}
    void wiring() {}
};

int main(int argc, char **argv) {
    cxxopts::Options options("ReactionOrder", "Multiport source connecting to banked sink reactors");

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
    
    auto src = new Src("src", &sim);
    auto sink = new Sink("sink", &sim);
    src->out --> sink->in;

    sim.run();
    return 0;
}
