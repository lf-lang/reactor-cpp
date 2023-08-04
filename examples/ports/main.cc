#include <iostream>

#include <reactor-cpp/reactor-cpp.hh>

using namespace reactor;
using namespace std::chrono_literals;

class Trigger : public Reactor {
private:
  Timer timer;
  Reaction r_timer{"r_timer", 1, this, [this]() { on_timer(); }};

public:
  Trigger(const std::string& name, Environment* env, Duration period)
      : Reactor(name, env)
      , timer{"timer", this, period, Duration::zero()} {}

  Output<void> trigger{"trigger", this}; // NOLINT

  void assemble() override {
    r_timer.declare_trigger(&timer);
    r_timer.declare_antidependency(&trigger);
  }

  void on_timer() { trigger.set(); }
};

class Counter : public Reactor {
private:
  int value_{0};
  Reaction r_trigger{"r_trigger", 1, this, [this]() { on_trigger(); }};

public:
  Counter(const std::string& name, Environment* env)
      : Reactor(name, env) {}

  Input<void> trigger{"trigger", this}; // NOLINT
  Output<int> count{"count", this};     // NOLINT

  void assemble() override {
    std::cout << "assemble Counter" << std::endl << std::flush;
    r_trigger.declare_trigger(&trigger);
    r_trigger.declare_antidependency(&count);
  }

  void on_trigger() {
    value_ += 1;
    count.set(value_);
  }
};

class Printer : public Reactor {
private:
  Reaction r_value{"r_value", 1, this, [this]() { on_value(); }};

public:
  Input<int> value{"value", this};     // NOLINT
  Input<int> forward{"forward", this}; // NOLINT
  Printer(const std::string& name, Environment* env)
      : Reactor(name, env) {}

  void assemble() override {
    std::cout << "assemble Printer" << std::endl << std::flush;
    r_value.declare_trigger(&value);
  }

  void on_value() { std::cout << this->name() << ": " << *value.get() << std::endl; }
};

class Adder : public Reactor {
private:
  Reaction r_add{"r_add", 1, this, [this]() { add(); }};

public:
  Input<int> i1{"i1", this};    // NOLINT
  Input<int> i2{"i1", this};    // NOLINT
  Output<int> sum{"sum", this}; // NOLINT

  Adder(const std::string& name, Environment* env)
      : Reactor(name, env) {}

  void assemble() override {
    r_add.declare_trigger(&i1);
    r_add.declare_trigger(&i2);
    r_add.declare_antidependency(&sum);
  }

  void add() {
    if (i1.is_present() && i2.is_present()) {
      sum.set(*i1.get() + *i2.get());
      std::cout << "setting sum" << std::endl;
    }
  }
};

auto main() -> int {
  Environment env{4};

  Trigger trigger1{"t1", &env, 1s};
  Counter counter1{"c1", &env};
  Printer printer1{"p1", &env};

  env.draw_connection(trigger1.trigger, counter1.trigger, ConnectionProperties{});
  env.draw_connection(counter1.count, printer1.value, ConnectionProperties{});

  Trigger trigger2{"t2", &env, 2s};
  Counter counter2{"c2", &env};
  Printer printer2{"p2", &env};

  // trigger2.trigger.set_inward_binding(&counter2.trigger);
  // counter2.count.set_inward_binding(&printer2.value);
  env.draw_connection(trigger2.trigger, counter2.trigger, ConnectionProperties{});
  env.draw_connection(counter2.count, printer2.value, ConnectionProperties{});

  Adder add{"add", &env};
  Printer p_add{"p_add", &env};

  env.draw_connection(counter1.count, add.i1, ConnectionProperties{});
  env.draw_connection(counter2.count, add.i2, ConnectionProperties{});
  env.draw_connection(add.sum, p_add.forward, ConnectionProperties{ConnectionType::Delayed, 10s, nullptr});
  env.draw_connection(p_add.forward, p_add.value, ConnectionProperties{ConnectionType::Delayed, 5s, nullptr});

  std::cout << "assemble" << std::endl << std::flush;
  env.assemble();

  std::cout << "optimize" << std::endl << std::flush;
  auto thread = env.startup();
  thread.join();

  return 0;
}
