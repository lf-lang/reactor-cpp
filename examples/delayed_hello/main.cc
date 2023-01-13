#include <iostream>

#include "reactor-cpp/action.hh"
#include "reactor-cpp/connection.hh"
#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;
using namespace std::chrono_literals;

class Hello : public Reactor {
private:
  // actions
  Timer timer{"timer", this, 1s, 2s};
  ShutdownTrigger sa{"terminate", this};
  // connection

  Port<void> output_port{"r_input_port", PortType::Output, this};
  Port<void> input_port{"r_input_port", PortType::Input, this};
  Connection<void> connection{"r_connection", this, std::chrono::nanoseconds(0), &input_port, &output_port};
  // reactions
  Reaction r_hello{"r_hello", 1, this, [this]() { hello(); }};
  Reaction r_react{"r_react", 1, this, [this]() { react(); }};
  Reaction r_terminate{"r_terminate", 2, this, [this]() { terminate(); }};

public:
  explicit Hello(Environment* env)
      : Reactor("Hello", env) {}

  void assemble() override {
    r_hello.declare_trigger(&timer);
    // r_terminate.declare_trigger(&sa);
  }

  static void hello() { std::cout << "Hello World!" << std::endl; }
  static void react() { std::cout << "React!" << std::endl; }
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
