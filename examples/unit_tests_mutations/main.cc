#include <iostream>

#include "../../lib/mutation/multiport.cc"
#include <reactor-cpp/mutations/multiport.hh>
#include <reactor-cpp/reactor-cpp.hh>

using namespace std::chrono_literals;

class TestMultiport: public reactor::Reactor {
  reactor::ModifableMultiport<reactor::Output<int>> test_multiport_{"modifable_multiports_"};
  reactor::Timer timer_{"timer", this, 1s};
  reactor::LogicalAction<int> scale{"scale", this};
  reactor::Reaction trigger_reaction_{"trigger_reaction", 1, this, [this](){this->__inner.reaction_1(scale);}};
  reactor::Reaction test_reaction_{"test_reaction", 2, this, [this](){this->__inner.reaction_2(test_multiport_, scale);}};
  reactor::Reaction validate_reaction_{"validate_reaction", 3, this, [this](){this->__inner.reaction_3(test_multiport_);}};
public:

class Inner: public reactor::MutableScope {
    int state = 0;
    std::vector<int> sizes = {4, 5, 6, 5, 4, 3};
    [[maybe_unused]] const Inner& __lf_inner = *this;
public:

    Inner(Reactor* reactor) : MutableScope(reactor) {}

    void reaction_1(reactor::LogicalAction<int>& scale) {
      int size = sizes[state];
      state = (state + 1) % sizes.size();
      std::cout << "set: " << size << std::endl;
      scale.schedule(size);
    }

    void reaction_2(reactor::ModifableMultiport<reactor::Output<int>>& test_multiport, reactor::LogicalAction<int>& scale) {
      reactor::ModifableMultiport<reactor::Output<int>>* temp = &test_multiport;
      std::size_t new_size = *scale.get();

      auto anti_dep = test_multiport[0].anti_dependencies();
      reactor::MutationChangeOutputMultiportSize change_size{temp, this->reactor_, anti_dep, new_size};
      add_to_transaction(&change_size);

      commit_transaction();
    }

    void reaction_3(reactor::ModifableMultiport<reactor::Output<int>>& test_multiport) {
      for (auto i = 0; i < test_multiport.size(); i++) {
        std::cout << test_multiport[i].fqn() << "/" << std::endl;
      }
    }
  };

  Inner __inner;

  TestMultiport(const std::string& name, reactor::Environment* env) : Reactor(name, env), __inner(this) {
    std::cout << "creating instance of deployment" << std::endl;

    test_multiport_.reserve(4);
    for (size_t _lf_idx = 0; _lf_idx < 4; _lf_idx++) {
      std::string _lf_port_name = test_multiport_.name() +  "_" + std::to_string(_lf_idx);
      test_multiport_.emplace_back(_lf_port_name, this);
    }
  }
  ~TestMultiport() override = default;

  void assemble() override {
    trigger_reaction_.declare_trigger(&timer_);
    trigger_reaction_.declare_schedulable_action(&scale);
    test_reaction_.declare_trigger(&scale);

  }
};


auto main() -> int {
  // srand(time(nullptr));
  reactor::Environment env{4};
  auto test_multiport = std::make_unique<TestMultiport>("test_multiport", &env);
  env.optimize();
  env.assemble();
  auto thread = env.startup();
  thread.join();
  return 0;
}
