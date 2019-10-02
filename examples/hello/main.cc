#include <iostream>

#include "dear/dear.hh"

#include <iostream>

using namespace dear;

class Hello : public Reactor {
 private:
  // actions
  Timer timer{"timer", this, 1_s, 2_s};

  // reactions
  Reaction r_hello{"hello", 1, this, [this]() { hello(); }};

 public:
  Hello(Environment* env) : Reactor("Hello", env) {}

  void assemble() override { r_hello.declare_trigger(&timer); }

  void hello() { std::cout << "Hello World!" << std::endl; }
};

int main() {
  Environment e{4};

  Hello hello{&e};
  e.assemble();
  e.init();

  auto t = e.start();
  t.join();

  return 0;
}
