#include <iostream>

#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;

class Count : public Reactor {
 private:
  // actions
  Timer timer{"timer", this};
  LogicalAction<int> counter{"counter", this};

  // reactions
  Reaction r_init{"r_init", 1, this, [this]() { init(); }};
  Reaction r_counter{"r_counter", 2, this, [this]() { print_count(); }};

 public:
  Count(Environment* env) : Reactor("Count", env) {}

  void assemble() override {
    r_init.declare_trigger(&timer);
    r_counter.declare_trigger(&counter);
  }

  void print_count() {
    auto& value = *(counter.get());
    std::cout << "Count: " << value << std::endl;
    counter.schedule(value + 1, 1_s);
  }

  void init() {
    std::cout << "Hello World!" << std::endl;
    counter.schedule(0, 1_s);
  }
};

int main() {
  Environment e{4};

  Count count{&e};
  e.assemble();
  e.init();

  auto t = e.start();
  t.join();

  return 0;
}
