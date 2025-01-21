/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef MULTIPORT_MUTATION_LOAD_BALANCER_HH
#define MULTIPORT_MUTATION_LOAD_BALANCER_HH

#include <reactor-cpp/mutations/multiport.hh>
#include <reactor-cpp/reactor-cpp.hh>

using namespace reactor;
using namespace std::chrono_literals;

class LoadBalancer final : public Reactor { // NOLINT
  class Inner : public MutableScope {
    explicit Inner(Reactor* reactor)
        : MutableScope(reactor) {}

    // reaction bodies
    static void reaction_1(const Input<unsigned>& inbound, Output<unsigned>& scale_bank,
                           Multiport<Output<unsigned>>& outbound) {
      if (std::rand() % 15 == 0) {            // NOLINT
        scale_bank.set(std::rand() % 20 + 1); // NOLINT
      }
      const unsigned outbound_port = std::rand() % outbound.size(); // NOLINT
      outbound[outbound_port].set(inbound.get());
    }

    friend LoadBalancer;
  };

  Inner _lf_inner;
  Reaction process{"process", 1, this, [this]() { Inner::reaction_1(this->inbound, this->scale_bank, this->out); }};

public:
  LoadBalancer(const std::string& name, Environment* env)
      : Reactor(name, env)
      , _lf_inner(this) {
    out.reserve(4);
    for (size_t _lf_idx = 0; _lf_idx < 4; _lf_idx++) {
      out.create_new_port();
    }
  }
  ~LoadBalancer() override = default;

  ModifableMultiport<Output<unsigned>> out{"out", this}; // NOLINT
  std::size_t out_size_ = 0;

  Input<unsigned> inbound{"inbound", this};        // NOLINT
  Output<unsigned> scale_bank{"scale_bank", this}; // NOLINT

  void assemble() override {
    for (auto& _lf_port : out) {
      process.declare_antidependency(&_lf_port);
    }
    process.declare_trigger(&inbound);
  }
};

#endif // MULTIPORT_MUTATION_LOAD_BALANCER_HH
