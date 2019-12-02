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
  Hello(Environment* env) : Reactor("Hello", env) {}

  void assemble() override {
    r_hello.declare_trigger(&timer);
    r_terminate.declare_trigger(&sa);
  }

  void hello() { std::cout << "Hello World!" << std::endl; }

  void terminate() { std::cout << "Good Bye!" << std::endl; }
};

class Timeout : public Reactor {
 private:
  Timer timer;

  Reaction r_timer{"r_timer", 1, this,
                   [this]() { environment()->sync_shutdown(); }};

 public:
  Timeout(Environment* env, Duration timeout)
      : Reactor("Timeout", env)
      , timer{"timer", this, Duration::zero(), timeout} {}

  void assemble() override { r_timer.declare_trigger(&timer); }
};

int main() {
  Environment e{4};

  Hello hello{&e};
  Timeout timeout{&e, 5s};
  e.assemble();

  auto t = e.startup();
  t.join();

  return 0;
}
