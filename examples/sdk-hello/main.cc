
#include <reactor-sdk/reactor-sdk.hh>

using namespace sdk;

class Hello : public Reactor {
private:
    Timer timer{"timer", this};

    void hello(Timer& timer) { std::cout << "Bank:" << bank_index << " Hello World!\n"; }
    void terminate(Shutdown& shutdown) { std::cout << "Good Bye!\n"; }

public:
    Hello(const std::string &name, Environment *env)
    : Reactor(name, env) {}

    void construction() {
        timer.set_timer (1s, 2s);
    }

    void assembling() override {
        reaction("reaction_1").
            triggers(&timer).
            dependencies().
            effects().
            function(
                [&](Timer& timer) {
                    std::cout << "Bank:" << bank_index << " Hello World!\n";
                }
            );
        
        reaction("reaction_2").
            triggers(&shutdown).
            dependencies().
            effects().
            function(pass_function(terminate));
    }
};

int main(int argc, char **argv) {
    Environment env {nullptr, 4, false, 4s, false};
    auto main = new Hello("Hello", &env);

    env.run();
    return 0;
}
