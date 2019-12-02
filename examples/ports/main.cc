#include <iostream>

#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;
using namespace std::chrono_literals;

class Trigger : public Reactor {
 private:
  Timer timer;

  Reaction r_timer{"r_timer", 1, this, [this]() { on_timer(); }};

 public:
  Trigger(const std::string& name, Environment* env, Duration period)
      : Reactor(name, env), timer{"timer", this, period, Duration::zero()} {}

  Output<void> trigger{"trigger", this};

  void assemble() override {
    r_timer.declare_trigger(&timer);
    r_timer.declare_antidependency(&trigger);
  }

  void on_timer() { trigger.set(); }
};

class Counter : public Reactor {
 private:
  unsigned value{0};

  Reaction r_trigger{"r_trigger", 1, this, [this]() { on_trigger(); }};

 public:
  Counter(const std::string& name, Environment* env) : Reactor(name, env) {}

  Input<void> trigger{"trigger", this};
  Output<int> count{"count", this};

  void assemble() override {
    r_trigger.declare_trigger(&trigger);
    r_trigger.declare_antidependency(&count);
  }

  void on_trigger() {
    value += 1;
    count.set(value);
  }
};

class Printer : public Reactor {
 private:
  Reaction r_value{"r_value", 1, this, [this]() { on_value(); }};

 public:
  Input<int> value{"value", this};

  Printer(const std::string& name, Environment* env) : Reactor(name, env) {}

  void assemble() override { r_value.declare_trigger(&value); }

  void on_value() {
    std::cout << this->name() << ": " << *value.get() << std::endl;
  }
};

class Adder : public Reactor {
 private:
  Reaction r_add{"r_add", 1, this, [this]() { add(); }};

 public:
  Input<int> i1{"i1", this};
  Input<int> i2{"i1", this};
  Output<int> sum{"sum", this};

  Adder(const std::string& name, Environment* env) : Reactor(name, env) {}

  void assemble() override {
    r_add.declare_trigger(&i1);
    r_add.declare_trigger(&i2);
    r_add.declare_antidependency(&sum);
  }

  void add() {
    if (i1.is_present() && i2.is_present()) {
      sum.set(*i1.get() + *i2.get());
    }
  }
};

int main() {
  Environment e{4};

  Trigger t1{"t1", &e, 1s};
  Counter c1{"c1", &e};
  Printer p1{"p1", &e};
  t1.trigger.bind_to(&c1.trigger);
  c1.count.bind_to(&p1.value);

  Trigger t2{"t2", &e, 2s};
  Counter c2{"c2", &e};
  Printer p2{"p2", &e};
  t2.trigger.bind_to(&c2.trigger);
  c2.count.bind_to(&p2.value);

  Adder add{"add", &e};
  Printer p_add{"p_add", &e};
  c1.count.bind_to(&add.i1);
  c2.count.bind_to(&add.i2);
  add.sum.bind_to(&p_add.value);

  e.assemble();

  auto t = e.startup();
  t.join();

  return 0;
}
