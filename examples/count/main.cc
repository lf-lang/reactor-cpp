#include <iostream>

#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;
using namespace std::chrono_literals;

class Count : public Reactor {
private:
  // actions
  Timer timer{"timer", this};
  LogicalAction<int> counter{"counter", this};

  // reactions_
  Reaction r_init{"r_init", 1, this, [this]() { init(); }};
  Reaction r_counter{"r_counter", 2, this, [this]() { print_count(); }};

public:
  explicit Count(Environment* env)
      : Reactor("Count", env) {}

  void assemble() override {
    r_init.declare_trigger(&timer);
    r_counter.declare_trigger(&counter);
  }

  void print_count() {
    const auto& value = *(counter.get());
    std::cout << "Count: " << value << std::endl;
    counter.schedule(value + 1, 1s);
  }

  void init() {
    std::cout << "Hello World!" << std::endl;
    counter.schedule(0, 1s);
  }
};

auto main() -> int {
  Environment env{4, true};

  Count count{&env};
  env.assemble();

  auto thread = env.startup();
  thread.join();

  return 0;
}
