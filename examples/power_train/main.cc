#include <iostream>

#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;

class LeftPedal : public Reactor {
 public:
  // ports
  Output<void> angle{"angle", this};
  Output<void> on_off{"on_off", this};

 private:
  // actions
  Action<void> req{"req", this};

  // reactions
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};

  void reaction_1() {}

 public:
  LeftPedal(Environment* env) : Reactor("LP", env) {}

  void assemble() override {
    r1.declare_trigger(&req);
    r1.declare_antidependency(&angle);
    r1.declare_antidependency(&on_off);
  }
};

class RightPedal : public Reactor {
 public:
  // ports
  Output<void> angle{"angle", this};
  Input<void> check{"check", this};

 private:
  // actions
  Action<void> pol{"pol", this};

  // reactionns
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};
  Reaction r2{"2", 2, this, [this]() { reaction_2(); }};

  void reaction_1() {}
  void reaction_2() {}

 public:
  RightPedal(Environment* env) : Reactor("RP", env) {}

  void assemble() override {
    r1.declare_trigger(&pol);
    r1.declare_antidependency(&angle);

    r2.declare_trigger(&check);
    r2.declare_scheduable_action(&pol);
  }
};

class BrakeControl : public Reactor {
 public:
  // ports
  Input<void> angle{"angle", this};
  Output<void> force{"force", this};

 private:
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};

  void reaction_1() {}

 public:
  BrakeControl(Environment* env) : Reactor("BC", env) {}

  void assemble() override {
    r1.declare_trigger(&angle);
    r1.declare_antidependency(&force);
  }
};

class EngineControl : public Reactor {
 public:
  // ports
  Input<void> angle{"angle", this};
  Input<void> on_off{"on_off", this};
  Output<void> check{"check", this};
  Output<void> torque{"torque", this};

 private:
  // actions
  Action<void> rev{"rev", this};

  // reactionns
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};
  Reaction r2{"2", 2, this, [this]() { reaction_2(); }};
  Reaction r3{"3", 3, this, [this]() { reaction_3(); }};

  void reaction_1() {}
  void reaction_2() {}
  void reaction_3() {}

 public:
  EngineControl(Environment* env) : Reactor("EC", env) {}

  void assemble() override {
    r1.declare_trigger(&on_off);
    r1.declare_antidependency(&torque);

    r2.declare_trigger(&angle);
    r2.declare_antidependency(&torque);

    r3.declare_trigger(&rev);
    r3.declare_antidependency(&check);
  }
};

class Brake : public Reactor {
 public:
  // ports
  Input<void> force{"force", this};

 private:
  // reactions
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};

  void reaction_1() {}

 public:
  Brake(Environment* env) : Reactor("B", env) {}

  void assemble() override { r1.declare_trigger(&force); }
};

class Engine : public Reactor {
 public:
  // ports
  Input<void> torque{"torque", this};

 private:
  // reactions
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};

  void reaction_1() {}

 public:
  Engine(Environment* env) : Reactor("E", env) {}

  void assemble() override { r1.declare_trigger(&torque); }
};

int main() {
  Environment e{4};

  LeftPedal left_pedal{&e};
  RightPedal right_pedal{&e};
  BrakeControl brake_control{&e};
  EngineControl engine_control{&e};
  Brake brakes{&e};
  Engine engine{&e};

  e.assemble();
  left_pedal.angle.bind_to(&brake_control.angle);
  left_pedal.on_off.bind_to(&engine_control.on_off);
  brake_control.force.bind_to(&brakes.force);
  right_pedal.angle.bind_to(&engine_control.angle);
  engine_control.check.bind_to(&right_pedal.check);
  engine_control.torque.bind_to(&engine.torque);

  e.init();
  e.export_dependency_graph("graph.dot");

  auto t = e.start();
  t.join();

  return 0;
}
