#include <iostream>

#include "reactor-cpp/action.hh"
#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;
using namespace std::chrono_literals;

class Hello : public Reactor {
private:
  // actions
  Timer timer{"timer", this, 1s, 2s};
  ShutdownTrigger sa{"terminate", this};

  // reactions
  Reaction r_hello{"r_hello", 1, this, [this]() { hello(); }};
  Reaction r_terminate{"r_terminate", 2, this, [this]() { terminate(); }};

public:
  explicit Hello(Environment* env)
      : Reactor("Hello", env) {}

  void assemble() override {
    r_hello.declare_trigger(&timer);
    r_terminate.declare_trigger(&sa);
  }

  static void hello() { std::cout << "Hello World!" << std::endl; }

  static void terminate() { std::cout << "Good Bye!" << std::endl; }
};

auto main() -> int {
  Environment env{4, false, false, 5s};

  Hello hello{&env};
  env.assemble();

  auto thread = env.startup();
  thread.join();

  return 0;
}
