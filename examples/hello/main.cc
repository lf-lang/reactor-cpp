#include <iostream>

#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;
using namespace std::chrono_literals;

class Hello : public Reactor {
private:
  // actions
  Timer timer{"timer", this, 1s, 2s};
  ShutdownAction sa{"terminate", this};

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

class Timeout : public Reactor {
private:
  Timer timer;

  Reaction r_timer{"r_timer", 1, this, [this]() { environment()->sync_shutdown(); }};

public:
  Timeout(Environment* env, Duration timeout)
      : Reactor("Timeout", env)
      , timer{"timer", this, Duration::zero(), timeout} {}

  void assemble() override { r_timer.declare_trigger(&timer); }
};

auto main() -> int {
  Environment env{4};

  Hello hello{&env};
  Timeout timeout{&env, 5s};
  env.assemble();

  auto thread = env.startup();
  thread.join();

  return 0;
}
