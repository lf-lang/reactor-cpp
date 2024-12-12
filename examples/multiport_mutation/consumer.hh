//
// Created by tanneberger on 11/17/24.
//

#ifndef CONSUMER_HH
#define CONSUMER_HH

#include <reactor-cpp/reactor-cpp.hh>

using namespace reactor;
using namespace std::chrono_literals;

class Consumer : public Reactor {
private:
  class Inner: public Scope {
    Inner(Reactor* reactor, std::size_t index) : Scope(reactor), index_(index) {}
    std::size_t index_;

    [[maybe_unused]] const Inner& __lf_inner = *this;

    void reaction_1([[maybe_unused]] const Input<unsigned>& in) {
      std::cout << "consumer: " << index_ << " received value:" << *in.get() << std::endl;
    }

    friend Consumer;
  };

  Inner __lf_inner;
  Reaction handle{"handle", 1, this, [this]() { __lf_inner.reaction_1(this->in); }};
public:
  Consumer(const std::string& name, Environment* env, std::size_t index) : Reactor(name, env), __lf_inner(this, index) {
    std::cout << "creating instance of consumer" << std::endl;
  }
  ~Consumer() override {
    std::cout << "Consumer Object is deleted" << std::endl;
  };

  Input<unsigned> in{"in", this};

  void assemble() override {
    handle.declare_trigger(&in);
  }
};


#endif //CONSUMER_HH
