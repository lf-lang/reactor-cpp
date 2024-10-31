

#include "reactor-cpp/transaction.hh"

template<class T>
reactor::MutationChangeMultiportSize<T>::MutationChangeMultiportSize(Multiport<T>* multiport, std::size_t size)
    : multiport_(multiport), desired_size_(size){
}

template<class T>
void reactor::MutationChangeMultiportSize<T>::run() {
  multiport_->ports_.resize(desired_size_);
}

auto reactor::Transaction::execute() -> MutationResult {
  for (auto *mutation : mutations_) {
    mutation->run();
  }

  return Success;
}

void reactor::Transaction::reset() {
    mutations_.clear();
}