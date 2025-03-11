
#include <reactor-sdk/reactor-sdk.hh>

using namespace sdk;

class Hello : public Reactor {
private:
    struct Parameters {
    };
    Parameters parameters;

    Timer timer{"timer", this};

    void hello(Timer& timer) { std::cout << "Bank:" << bank_index << " Hello World!\n"; }
    // void terminate(Shutdown& shutdown) { std::cout << "Good Bye!\n"; }

    REACTION_SCOPE_START(Hello, Parameters)

        void terminate(Shutdown& shutdown) {
            std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                        << fqn() << " Good Bye!\n";
        }
        void add_reactions (Hello *reactor) {
            reaction("reaction_1").
                triggers(&reactor->timer).
                dependencies().
                effects().
                function(
                    [&](Timer& timer) {
                        std::cout   << "(" << get_elapsed_logical_time().count() << ", " << get_microstep() << ") physical_time:" << get_elapsed_physical_time().count()
                        << fqn() << " Bank:" << bank_index << " Hello World!\n";
                    }
                );

            reaction("reaction_2").
                triggers(&reactor->shutdown).
                dependencies().
                effects().
                function(pass_function(terminate));
        }

    REACTION_SCOPE_END(this, parameters)

public:
    Hello(const std::string &name, Environment *env)
    : Reactor(name, env) {}

    void construction() {
        timer.set_timer (1s, 2s);
    }

    void wiring() override {
        
    }
};

int main(int argc, char **argv) {
    Environment env {nullptr, 4, false, 4s, false};
    auto main = new Hello("Hello", &env);

    env.run();
    return 0;
}
