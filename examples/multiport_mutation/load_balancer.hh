#ifndef LOAD_BALANCER_HH // NOLINT
#define LOAD_BALANCER_HH // NOLINT

#include <reactor-cpp/reactor-cpp.hh>

#include "../../lib/mutation/multiport.cc"
#include "reactor-cpp/mutations/multiport.hh"

using namespace reactor;
using namespace std::chrono_literals;

class LoadBalancer final : public Reactor { // NOLINT
  class Inner : public MutableScope {
    explicit Inner(Reactor* reactor)
        : MutableScope(reactor) {}

    // reaction bodies
    static void reaction_1(const Input<unsigned>& inbound, LogicalAction<unsigned>& scale_action,
                           Multiport<Output<unsigned>>& outbound) {
      if (std::rand() % 30 == 0) {                   // NOLINT
        scale_action.schedule(std::rand() % 20 + 1); // NOLINT
      }
      const unsigned sel = std::rand() % outbound.size(); // NOLINT
      std::cout << "Sending out to:" << sel << '\n';
      outbound[sel].set(inbound.get());
    }

    void reaction_2(ModifableMultiport<Output<unsigned>>& outbound,
                    [[maybe_unused]] const LogicalAction<unsigned>& scale, Output<unsigned>& scale_bank) {
      ModifableMultiport<Output<unsigned>>* temp = &outbound;
      std::size_t new_size = *scale.get();

      auto antideps = (outbound[0]).anti_dependencies();

      const auto change_size =
          std::make_shared<MutationChangeOutputMultiportSize<unsigned>>(temp, this->reactor_, antideps, new_size);

      add_to_transaction(change_size);

      commit_transaction();

      scale_bank.set(new_size);
    }

    friend LoadBalancer;
  };

  Inner _lf_inner;
  Reaction process{"process", 2, this, [this]() { Inner::reaction_1(this->inbound, this->scale_action, this->out); }};
  Reaction scale{"scale", 1, this, [this]() { _lf_inner.reaction_2(this->out, this->scale_action, this->scale_bank); }};

public:
  LoadBalancer(const std::string& name, Environment* env)
      : Reactor(name, env)
      , _lf_inner(this) {
    std::cout << "creating instance of load balancer" << '\n';
    out.reserve(4);
    for (size_t _lf_idx = 0; _lf_idx < 4; _lf_idx++) {
      std::string _lf_port_name = out.name() + "_" + std::to_string(_lf_idx);
      out.emplace_back(_lf_port_name, this);
    }
  }
  ~LoadBalancer() override = default;

  LogicalAction<unsigned> scale_action{"scale", this, 1us}; // NOLINT
  ModifableMultiport<Output<unsigned>> out{"out"};          // NOLINT
  Input<unsigned> inbound{"inbound", this};                 // NOLINT
  Output<unsigned> scale_bank{"scale_bank", this};          // NOLINT

  void assemble() override {
    std::cout << "assemble LoadBalancer\n";
    for (auto& _lf_port : out) {
      process.declare_antidependency(&_lf_port);
    }
    process.declare_trigger(&inbound);
    scale.declare_trigger(&scale_action);
    scale.declare_antidependency(&scale_bank);
  }
};

#endif // LOAD_BALANCER_HH
