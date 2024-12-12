#include <iostream>

#include <reactor-cpp/mutations/bank.hh>
#include <reactor-cpp/mutations/connection.hh>

#include "./consumer.hh"
#include "./load_balancer.hh"
#include "./producer.hh"
#include "../../lib/mutation/bank.cc"
#include "../../lib/mutation/connection.cc"
#include <reactor-cpp/reactor-cpp.hh>

class Deployment : public Reactor {
  std::unique_ptr<Producer> producer_;
  std::unique_ptr<LoadBalancer> load_balancer_;
  std::vector<std::unique_ptr<Consumer>> consumers_;

  Reaction scale_bank{"scale_bank", 1, this, [this](){this->__inner.reaction_1(this->scale, this->consumers_, load_balancer_->out);}};

public:

class Inner: public MutableScope {
    int state = 0;
    [[maybe_unused]] const Inner& __lf_inner = *this;
public:

    Inner(Reactor* reactor) : MutableScope(reactor) {}
    void reaction_1(const Input<unsigned>& scale, std::vector<std::unique_ptr<Consumer>>& reactor_bank, ModifableMultiport<Output<unsigned>>& load_balancer) {
      std::size_t new_size = *scale.get();
      std::size_t old_size = reactor_bank.size();

      std::function<std::unique_ptr<Consumer>(Reactor*, std::size_t)> lambda = [](Reactor* reactor, std::size_t index) {
        std::string __lf_inst_name = "consumer_" + std::to_string(index);
        return std::make_unique<Consumer>(__lf_inst_name, reactor->environment(), index);
      };
      MutationChangeBankSize change_size{&reactor_bank, this->reactor_, new_size, lambda};

      add_to_transaction(&change_size);

      // old topology
      commit_transaction();
      // new topology

      if (old_size > new_size) {

        for (auto i = 0; i < old_size - new_size; i++) {
        }
      } else {
        std::cout << "load_balancer size:" << load_balancer.size() << " bank size: " << reactor_bank.size() << std::endl;
        for (auto i = 0; i < new_size; i++) {
            std::cout << "add connection: " << i << std::endl;
            MutationAddConnection<Output<unsigned>, Input<unsigned>> add_conn{&load_balancer[i], &reactor_bank[i].get()->in, reactor_};
            add_to_transaction(&add_conn);
        }
        commit_transaction(true);
      }

      std::cout << "new bank size:" << reactor_bank.size() << std::endl;
    }

    friend LoadBalancer;
  };

  Inner __inner;

  Deployment(const std::string& name, Environment* env) : Reactor(name, env), __inner(this),
  producer_(std::make_unique<Producer>("producer", environment())),
  load_balancer_(std::make_unique<LoadBalancer>("load_balancer", environment())) {
    std::cout << "creating instance of deployment" << std::endl;
    consumers_.reserve(4);
    for (size_t __lf_idx = 0; __lf_idx < 4; __lf_idx++) {
      std::string __lf_inst_name = "consumer_" + std::to_string(__lf_idx);
      consumers_.push_back(std::make_unique<Consumer>(__lf_inst_name, environment(), __lf_idx));
    }
  }
  ~Deployment() override = default;

  Input<unsigned> scale{"scale", this};

  void assemble() override {
    for (size_t __lf_idx = 0; __lf_idx < 4; __lf_idx++) {
      environment()->draw_connection(load_balancer_->out[__lf_idx], consumers_[__lf_idx]->in, ConnectionProperties{});
      environment()->draw_connection(producer_->value, load_balancer_->inbound, ConnectionProperties{});
    }
    environment()->draw_connection(load_balancer_->scale_bank, scale, ConnectionProperties{});
    scale_bank.declare_trigger(&this->scale);
  }
};


auto main() -> int {
  //srand(time(nullptr));
  Environment env{4, true};
  auto deployment = std::make_unique<Deployment>("c1", &env);
  env.optimize();
  env.assemble();
  auto thread = env.startup();
  thread.join();
  return 0;
}
