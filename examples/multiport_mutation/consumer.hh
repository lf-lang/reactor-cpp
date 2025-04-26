/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef MULTIPORT_MUTATION_CONSUMER_HH
#define MULTIPORT_MUTATION_CONSUMER_HH

#include <reactor-cpp/reactor-cpp.hh>
#include <reactor-cpp/scopes.hh>

using namespace reactor;
using namespace std::chrono_literals;

class Consumer final : public Reactor { // NOLINT
  class Inner : public Scope {
    Inner(Reaction* reaction, std::size_t index)
        : Scope(reaction)
        , index_(index) {}
    std::size_t index_ = 0;

    void reaction_1(const Input<unsigned>& in) const {
      // std::cout << "consumer: " << index_ << " received value:" << *in.get() << '\n';
    }

    friend Consumer;
  };

  Inner _lf_inner;
  Reaction handle{"handle", 1, false, this, [this]() { _lf_inner.reaction_1(this->in); }};

public:
  Consumer(const std::string& name, Environment* env, std::size_t index)
      : Reactor(name, env)
      , _lf_inner(&handle, index) {
    std::cout << "creating instance of consumer" << '\n';
  }
  ~Consumer() override { std::cout << "Consumer Object is deleted" << '\n'; };

  Input<unsigned> in{"in", this}; // NOLINT

  void assemble() override { handle.declare_trigger(&in); }
};

#endif // MULTIPORT_MUTATION_CONSUMER_HH
