//
// Created by tanneberger on 11/17/24.
//

#ifndef PRODUCER_HH
#define PRODUCER_HH

#include <reactor-cpp/reactor-cpp.hh>

using namespace reactor;
using namespace std::chrono_literals;

class Producer : public Reactor {
private:
  Timer timer{"timer", this, 1s, 1s};
  Reaction r_timer{"r_timer", 1, this, [this]() { __lf_inner.reaction_1(this->value);}};

  class Inner: public Scope {
    unsigned itr = 0;
    [[maybe_unused]] const Inner& __lf_inner = *this;
    void reaction_1([[maybe_unused]] Output<unsigned>& out) {
      std::cout << "producing value:" << itr << std::endl;
      out.set(itr++);
    }
    explicit Inner(Reactor* reactor) : Scope(reactor) {}

    friend Producer;
  };

  Inner __lf_inner;
public:
  Producer(const std::string& name, Environment* env) : Reactor(name, env), __lf_inner(this) {
    std::cout << "creating instance of producer" << std::endl;
  }
  Producer() = delete;
  ~Producer() override = default;

  Output<unsigned> value{"value", this};

  void assemble() override {
    r_timer.declare_trigger(&timer);
    r_timer.declare_antidependency(&value);
  }
};

#endif //PRODUCER_HH
