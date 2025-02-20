
#include <reactor-sdk/reactor-sdk.hh>

using namespace std;
using namespace sdk;

class ActionIsPresent : public Reactor {
struct Parameters : public SystemParameter<Duration> {
        ParameterMetadata<Duration> offset = ParameterMetadata<Duration> {
            .name = "offset",
            .description = "offset",
            .min_value = 1ns,
            .max_value = 10ns,
            .value = 1ns
        };

        ParameterMetadata<Duration> period = ParameterMetadata<Duration> {
            .name = "period",
            .description = "period",
            .min_value = 500ms,
            .max_value = 1s,
            .value = 500ms
        };

        Parameters(Reactor *container)
            :   SystemParameter<Duration>(container) {
            register_parameters (offset, period);
        }
    };
private:
    Parameters parameters{this};
    LogicalAction<void> a{"a", this};
    bool success = false;
    Duration zero = 0ns;
public:
    ActionIsPresent(const std::string &name, Environment *env)
        : Reactor(name, env) {}
    ActionIsPresent(const std::string &name, Reactor *container)
        : Reactor(name, container) {}
    
    void construction() {
    }

    void assembling() {
        reaction ("reaction_1").
            triggers(&startup, &a).
            effects().
            function (
                [&](Startup &startup, LogicalAction<void> &a) {
                    if (!a.is_present()) {
                        if (parameters.offset.value == zero) {
                            std::cout << "Hello World!" << '\n';
                            success = true;
                        } else {
                            a.schedule(parameters.offset.value);
                        }
                    } else {
                        std::cout << "Hello World 2!" << '\n';
                        success = true;
                    }
                }
            );
        
        reaction ("reaction_2").
            triggers(&shutdown).
            effects().
            function (
                [&](Shutdown &shutdown) {
                    if (!success) {
                        std::cerr << "Failed to print 'Hello World!'" << '\n';
                        exit(1);
                    }
                }
            );
    }
};

int main(int argc, char **argv) {
    cxxopts::Options options("ActionIsPresent", "Multiport source connecting to banked sink reactors");

    unsigned workers = std::thread::hardware_concurrency();
    bool fast{false};
    reactor::Duration timeout = reactor::Duration::max();
    bool visualize{false};

    // the timeout variable needs to be tested beyond fitting the Duration-type 
    options
    .set_width(120)
    .add_options()
        ("w,workers", "the number of worker threads used by the scheduler", cxxopts::value<unsigned>(workers)->default_value(std::to_string(workers)), "'unsigned'")
        ("o,timeout", "Time after which the execution is aborted.", cxxopts::value<reactor::Duration>(timeout)->default_value(time_to_string(timeout)), "'FLOAT UNIT'")
        ("f,fast", "Allow logical time to run faster than physical time.", cxxopts::value<bool>(fast)->default_value("false"))
        ("v,visualize", "Generate graph.dot file of the topology.", cxxopts::value<bool>(visualize)->default_value("false"))
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

    std::cout << "parameters - workers:" << workers << " fast:" << (fast ? "True" : "False") << " timeout:" << timeout << " visualize:" << (visualize ? "True" : "False") << std::endl;

    Environment sim {nullptr, workers, fast, timeout, visualize};
    auto action_delay = new ActionIsPresent("ActionIsPresent", &sim);

    sim.run();
    return 0;
}
