#include <iostream>

#include <reactor-cpp/mutations/bank.hh>
#include <reactor-cpp/mutations/connection.hh>

#include "../../lib/mutation/bank.cc"
#include "../../lib/mutation/connection.cc"
#include "./consumer.hh"
#include "./load_balancer.hh"
#include "./producer.hh"
#include <reactor-cpp/reactor-cpp.hh>

class Deployment final : public Reactor { // NOLINT
  class Inner : public MutableScope {
    int state = 0;

  public:
    Inner(Reactor* reactor)
        : MutableScope(reactor) {}
    void reaction_1(const Input<unsigned>& scale, std::vector<std::unique_ptr<Consumer>>& reactor_bank,
                    ModifableMultiport<Output<unsigned>>& load_balancer) {
      std::size_t new_size = *scale.get();
      std::size_t old_size = reactor_bank.size();

      std::function lambda = [](Reactor* reactor, std::size_t index) {
        std::string _lf_inst_name = "consumer_" + std::to_string(index);
        return std::make_unique<Consumer>(_lf_inst_name, reactor->environment(), index);
      };

      auto change_size = std::make_shared<MutationChangeBankSize<std::unique_ptr<Consumer>>>(
          &reactor_bank, this->reactor_, new_size, lambda);

      add_to_transaction(change_size);

      commit_transaction();

      if (old_size < new_size) {
        for (auto i = 0; i < new_size; i++) {
          auto add_conn = std::make_shared<MutationAddConnection<Output<unsigned>, Input<unsigned>>>(
              &load_balancer[i], &reactor_bank[i].get()->in, reactor_);
          add_to_transaction(add_conn);
        }
      }

      commit_transaction(true);
    }

    friend LoadBalancer;
  };

  std::unique_ptr<Producer> producer_;
  std::unique_ptr<LoadBalancer> load_balancer_;
  std::vector<std::unique_ptr<Consumer>> consumers_;

  Reaction scale_bank{"scale_bank", 1, this,
                      [this]() { this->_inner.reaction_1(this->scale, this->consumers_, load_balancer_->out); }};

  Inner _inner;

public:
  Deployment(const std::string& name, Environment* env)
      : Reactor(name, env)
      , _inner(this)
      , producer_(std::make_unique<Producer>("producer", environment()))
      , load_balancer_(std::make_unique<LoadBalancer>("load_balancer", environment())) {
    std::cout << "creating instance of deployment" << '\n';
    consumers_.reserve(4);
    for (size_t _lf_idx = 0; _lf_idx < 4; _lf_idx++) {
      std::string _lf_inst_name = "consumer_" + std::to_string(_lf_idx);
      consumers_.push_back(std::make_unique<Consumer>(_lf_inst_name, environment(), _lf_idx));
    }
  }
  ~Deployment() override = default;

  Input<unsigned> scale{"scale", this}; // NOLINT

  void assemble() override {
    for (size_t _lf_idx = 0; _lf_idx < 4; _lf_idx++) {
      environment()->draw_connection(load_balancer_->out[_lf_idx], consumers_[_lf_idx]->in, ConnectionProperties{});
      environment()->draw_connection(producer_->value, load_balancer_->inbound, ConnectionProperties{});
    }
    environment()->draw_connection(load_balancer_->scale_bank, scale, ConnectionProperties{});
    scale_bank.declare_trigger(&this->scale);
  }
};

auto main() -> int {
  Environment env{4, true};
  auto deployment = std::make_unique<Deployment>("c1", &env);
  env.optimize();
  env.assemble();
  auto thread = env.startup();
  thread.join();
  return 0;
}