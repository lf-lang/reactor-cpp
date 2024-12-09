

#include "reactor-cpp/transaction.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/reactor.hh"

 reactor::Transaction::Transaction(Reactor* parent)  : parent_(parent), environment_(parent->environment()) {  }


auto reactor::Transaction::execute() -> MutationResult {

   this->environment_->start_mutation();
   for (auto *mutation : mutations_) {
     mutation->run();
   }

   this->environment_->stop_mutation();
   mutations_.clear();
    return Success;
}

void reactor::Transaction::push_back(reactor::Mutation* mutation) {
    mutations_.push_back(mutation);
}