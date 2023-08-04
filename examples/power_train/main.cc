#include <iostream>

#include "reactor-cpp/reactor-cpp.hh"

using namespace reactor;

class LeftPedal : public Reactor {
public:
  // ports
  Output<void> angle{"angle", this};   // NOLINT
  Output<void> on_off{"on_off", this}; // NOLINT

private:
  // actions
  PhysicalAction<void> req{"req", this};

  // reactions
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};

  void reaction_1() {}

public:
  explicit LeftPedal(Environment* env)
      : Reactor("LP", env) {}

  void assemble() override {
    r1.declare_trigger(&req);
    r1.declare_antidependency(&angle);
    r1.declare_antidependency(&on_off);
  }
};

class RightPedal : public Reactor {
public:
  // ports
  Output<void> angle{"angle", this}; // NOLINT
  Input<void> check{"check", this};  // NOLINT

private:
  // actions
  PhysicalAction<void> pol{"pol", this};

  // reactions
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};
  Reaction r2{"2", 2, this, [this]() { reaction_2(); }};

  void reaction_1(){};
  void reaction_2() {}

public:
  explicit RightPedal(Environment* env)
      : Reactor("RP", env) {}

  void assemble() override {
    r1.declare_trigger(&pol);
    r1.declare_antidependency(&angle);

    r2.declare_trigger(&check);
    r2.declare_schedulable_action(&pol);
  }
};

class BrakeControl : public Reactor {
public:
  // ports
  Input<void> angle{"angle", this};  // NOLINT
  Output<void> force{"force", this}; // NOLINT

private:
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};

  void reaction_1() {}

public:
  explicit BrakeControl(Environment* env)
      : Reactor("BC", env) {}

  void assemble() override {
    r1.declare_trigger(&angle);
    r1.declare_antidependency(&force);
  }
};

class EngineControl : public Reactor {
public:
  // ports
  Input<void> angle{"angle", this};    // NOLINT
  Input<void> on_off{"on_off", this};  // NOLINT
  Output<void> check{"check", this};   // NOLINT
  Output<void> torque{"torque", this}; // NOLINT

private:
  // actions
  PhysicalAction<void> rev{"rev", this};

  // reactions
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};
  Reaction r2{"2", 2, this, [this]() { reaction_2(); }};
  Reaction r3{"3", 3, this, [this]() { reaction_3(); }};

  void reaction_1() {}
  void reaction_2() {}
  void reaction_3() {}

public:
  explicit EngineControl(Environment* env)
      : Reactor("EC", env) {}

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
  Input<void> force{"force", this}; // NOLINT

private:
  // reactions_
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};

  void reaction_1() {}

public:
  explicit Brake(Environment* env)
      : Reactor("B", env) {}

  void assemble() override { r1.declare_trigger(&force); }
};

class Engine : public Reactor {
public:
  // ports
  Input<void> torque{"torque", this}; // NOLINT

private:
  // reactions_
  Reaction r1{"1", 1, this, [this]() { reaction_1(); }};

  void reaction_1() {}

public:
  explicit Engine(Environment* env)
      : Reactor("E", env) {}

  void assemble() override { r1.declare_trigger(&torque); }
};

auto main() -> int {
  Environment env{4};

  LeftPedal left_pedal{&env};
  RightPedal right_pedal{&env};
  BrakeControl brake_control{&env};
  EngineControl engine_control{&env};
  Brake brakes{&env};
  Engine engine{&env};

  env.draw_connection(left_pedal.angle, brake_control.angle, ConnectionProperties{});
  env.draw_connection(left_pedal.angle, engine_control.on_off, ConnectionProperties{});
  env.draw_connection(brake_control.force, brakes.force, ConnectionProperties{});
  env.draw_connection(right_pedal.angle, engine_control.angle, ConnectionProperties{});
  env.draw_connection(engine_control.check, right_pedal.check, ConnectionProperties{});
  env.draw_connection(engine_control.torque, engine.torque, ConnectionProperties{});

  env.assemble();

  env.export_dependency_graph("graph.dot");

  auto thread = env.startup();
  thread.join();

  return 0;
}
