#ifndef PRODUCER_HH // NOLINT
#define PRODUCER_HH // NOLINT

#include <reactor-cpp/reactor-cpp.hh>

using namespace reactor;
using namespace std::chrono_literals;

class Producer final : public Reactor { // NOLINT
private:
  Timer timer{"timer", this, 1s, 1s};
  Reaction r_timer{"r_timer", 1, this, [this]() { _lf_inner.reaction_1(this->value); }};

  class Inner : public Scope {
    unsigned int counter_ = 0;

    void reaction_1([[maybe_unused]] Output<unsigned>& out) {
      // std::cout << "producing value:" << counter_ << "\n";
      out.set(counter_++);
    }

    explicit Inner(Reactor* reactor)
        : Scope(reactor) {}

    friend Producer;
  };

  Inner _lf_inner;

public:
  Producer(const std::string& name, Environment* env)
      : Reactor(name, env)
      , _lf_inner(this) {
    std::cout << "creating instance of producer\n";
  }
  Producer() = delete;
  ~Producer() override = default;

  Output<unsigned> value{"value", this}; // NOLINT

  void assemble() override {
    r_timer.declare_trigger(&timer);
    r_timer.declare_antidependency(&value);
  }
};

#endif // PRODUCER_HH
