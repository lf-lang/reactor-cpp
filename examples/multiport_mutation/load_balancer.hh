//
// Created by tanneberger on 11/17/24.
//

#ifndef LOAD_BALANCER_HH
#define LOAD_BALANCER_HH

#include <reactor-cpp/reactor-cpp.hh>

#include "../../lib/mutation/multiport.cc"
#include "reactor-cpp/mutations/multiport.hh"

using namespace reactor;
using namespace std::chrono_literals;

class LoadBalancer : public Reactor {
private:
  class Inner: public MutableScope {
    Inner(Reactor* reactor) : MutableScope(reactor) {}
    [[maybe_unused]] const Inner& __lf_inner = *this;

    // reaction bodies
    void reaction_1(const Input<unsigned>& inbound, LogicalAction<unsigned>& scale_action, Multiport<Output<unsigned>>& outbound) {
      if (rand() % 30 == 0) {
        scale_action.schedule(rand() % 20 + 1);
      }
      unsigned sel = rand() % outbound.size();
      std::cout << "Sending out to:" << sel << std::endl;
      outbound[sel].set(inbound.get());
    }

    void reaction_2(ModifableMultiport<Output<unsigned>>&outbound, [[maybe_unused]] const LogicalAction<unsigned>& scale, Output<unsigned>& scale_bank) {
      ModifableMultiport<Output<unsigned>>* temp = &outbound;
      std::size_t new_size = *scale.get();

      auto antideps = (outbound[0]).anti_dependencies();

      MutationChangeOutputMultiportSize change_size{temp, this->reactor_, antideps, new_size};

      add_to_transaction(&change_size);

      commit_transaction();

      scale_bank.set(new_size);
    }

    friend LoadBalancer;
  };

  Inner __lf_inner;
  Reaction process{"process", 2, this, [this]() { __lf_inner.reaction_1(this->inbound, this->scale_action, this->out); }};
  Reaction scale{"scale", 1, this, [this]() { __lf_inner.reaction_2(this->out, this->scale_action, this->scale_bank); }};

public:
  LoadBalancer(const std::string& name, Environment* env)
      : Reactor(name, env), __lf_inner(this) {
    std::cout << "creating instance of load balancer" << std::endl;
    out.reserve(4);
    for (size_t _lf_idx = 0; _lf_idx < 4; _lf_idx++) {
      std::string _lf_port_name = out.name() +  "_" + std::to_string(_lf_idx);
      out.emplace_back(_lf_port_name, this);
    }
  }

  LogicalAction<unsigned> scale_action{"scale", this, 1us};
  ModifableMultiport<Output<unsigned>> out{"out"};
  Input<unsigned> inbound{"inbound", this}; // NOLINT
  Output<unsigned> scale_bank{"scale_bank", this};

  void assemble() override {
    std::cout << "assemble LoadBalancer\n";
    for (auto& __lf_port : out) {
      process.declare_antidependency(&__lf_port);
    }
    process.declare_trigger(&inbound);
    scale.declare_trigger(&scale_action);
    scale.declare_antidependency(&scale_bank);
  }
};



#endif //LOAD_BALANCER_HH
