

#include "reactor-cpp/transaction.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/reactor.hh"

reactor::Transaction::Transaction(Reactor* parent)
    : environment_(parent->environment())
    , parent_(parent) {}

auto reactor::Transaction::execute(bool recalculate) -> MutationResult {

  this->environment_->start_mutation();
  for (const auto& mutation : mutations_) {
    mutation->run();
  }

  if (recalculate) {
    // parent_->remove_dependency_graph();
    // this->environment_->clear_dependency_graph();
    // this->environment_->build_dependency_graph(this->parent_);

    // this->environment_
    // for (auto* reactor : this->environment_->top_level_reactors()) {
    //   this->environment_->build_dependency_graph(reactor);
    // }

    this->environment_->calculate_indexes();
  }

  this->environment_->stop_mutation();
  this->environment_->export_dependency_graph("./test.dot");
  mutations_.clear();
  return Success;
}

void reactor::Transaction::push_back(const std::shared_ptr<reactor::Mutation>& mutation) {
  mutations_.push_back(mutation);
}