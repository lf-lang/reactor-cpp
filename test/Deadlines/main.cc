
#include <reactor-sdk/reactor-sdk.hh>

using namespace std;
using namespace sdk;

class Source : public Reactor {
    struct Parameters : public SystemParameter<Duration> {
        REACTOR_PARAMETER (Duration, period, "period", 1s, 5s, 2s);

        Parameters(Reactor *container)
            :   SystemParameter<Duration>(container) {
            register_parameters (period);
        }
    };
    Parameters parameters{this};
    Timer t{"t", this};
    int count = 0;
public:
    Source(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    Source(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    Output<int> y{"y", this};

    void construction() {
        t.set_timer (parameters.period.value, 0ns);
    }

    void assembling() {
        reaction ("reaction_1").
            triggers(&t).
            effects(&y).
            function (
                [&](Timer &t, Output<int> &y) {
                    if (count % 2 == 1) {
                        // The count variable is odd.
                        // Take time to cause a deadline violation.
                        std::this_thread::sleep_for(400ms);
                    }
                    std::cout << "Source sends: " << count << std::endl;
                    y.set(count);
                    count++;
                }
            );
    }
};

class Destination : public Reactor {
    struct Parameters : public SystemParameter<Duration> {
        REACTOR_PARAMETER (Duration, timeout, "timeout", 100ms, 1s, 200ms);

        Parameters(Reactor *container)
            :   SystemParameter<Duration>(container) {
            register_parameters (timeout);
        }
    };
    Parameters parameters{this};
    int count = 0;
public:
    Destination(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    Destination(const std::string &name, Reactor *container)
        : Reactor(name, container) {}

    Input<int> x{"x", this};

    void construction() {}

    void assembling() {
        reaction ("reaction_1").
            triggers(&x).
            effects().
            function (
                [&](Input<int> &x) {
                    std::cout << "Destination receives: " << *x.get() << std::endl;
                    if (count % 2 == 1) {
                        // The count variable is odd, so the deadline should have been
                        // violated
                        std::cerr << "ERROR: Failed to detect deadline." << std::endl;
                        exit(1);
                    }
                    count++;
                }
            ).deadline (parameters.timeout.value,
                [&](Input<int> &x) {
                    std::cout << "Destination deadline handler receives: "
                        << *x.get() << std::endl;
                    if (count % 2 == 0) {
                        // The count variable is even, so the deadline should not have
                        // been violated.
                        std::cerr << "ERROR: Deadline handler invoked without deadline "
                                << "violation." << std::endl;
                        exit(2);
                    }
                    count++;
                });
    }
};

class Deadline : public Reactor {
    std::unique_ptr<Source> s;
    std::unique_ptr<Destination> d;
public:
    Deadline(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    Deadline(const std::string &name, Reactor *container)
        : Reactor(name, container) {}
    
    void construction() {
        s = std::make_unique<Source>("Source", this);
        d = std::make_unique<Destination>("Destination", this);
    }

    void assembling() {
        s->y --> d->x;
    }
};

int main(int argc, char **argv) {
    cxxopts::Options options("Deadlines", "Multiport source connecting to banked sink reactors");

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
    auto dl = new Deadline("Deadline", &sim);

    sim.run();
    return 0;
}
