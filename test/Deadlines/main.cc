
#include <reactor-sdk/reactor-sdk.hh>

using namespace std;
using namespace sdk;

class Source : public Reactor {
    struct Parameters : public SystemParameter<Duration> {
        ParameterMetadata<Duration> period = ParameterMetadata<Duration> {
            .name = "period",
            .description = "period",
            .min_value = 1s,
            .max_value = 5s,
            .value = 2s
        };

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
        ParameterMetadata<Duration> timeout = ParameterMetadata<Duration> {
            .name = "timeout",
            .description = "timeout",
            .min_value = 100ms,
            .max_value = 1s,
            .value = 200ms
        };

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
    unsigned workers = 1;
    bool fast{false};
    reactor::Duration timeout = 4s;

    bool visualize = false;

    if (argc > 1) {
        string v_str = argv[1];
        visualize =  (v_str == "true") ? true : false;
    }

    std::cout << "parameters - workers:" << workers << " fast:" << (fast ? "True" : "False") << " timeout:" << timeout << " visualize:" << visualize << std::endl;

    Environment sim {nullptr, workers, fast, timeout, visualize};
    auto dl = new Deadline("Deadline", &sim);

    sim.run();
    return 0;
}
