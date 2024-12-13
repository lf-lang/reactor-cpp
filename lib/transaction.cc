

#include "reactor-cpp/transaction.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/reactor.hh"

 reactor::Transaction::Transaction(Reactor* parent) : environment_(parent->environment()) {  }


auto reactor::Transaction::execute(bool recalculate) -> MutationResult {

   this->environment_->start_mutation();
   for (auto mutation : mutations_) {
     mutation->run();
   }

   if (recalculate) {
    this->environment_->clear_dependency_graph();
      for (const auto* reactor : this->environment_->top_level_reactors()) {
        this->environment_->build_dependency_graph((Reactor*)reactor);
      }

      this->environment_->calculate_indexes();
   }

   this->environment_->stop_mutation();

   mutations_.clear();
   return Success;
}

void reactor::Transaction::push_back(const std::shared_ptr<reactor::Mutation>& mutation) {
    mutations_.push_back(mutation);
}