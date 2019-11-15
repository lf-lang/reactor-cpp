#include <iostream>

#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;

class Hello : public Reactor {
 private:
  // actions
  Timer timer{"timer", this, 1_s, 2_s};
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

int main() {
  Environment e{4};

  Hello hello{&e};
  e.assemble();

  auto t = e.startup();
  std::this_thread::sleep_for(std::chrono::seconds(5));
  e.async_shutdown();
  t.join();

  return 0;
}
