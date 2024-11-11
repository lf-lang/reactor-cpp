

#include "reactor-cpp/transaction.hh"

template<class T>
reactor::MutationChangeMultiportSize<T>::MutationChangeMultiportSize(Multiport<T>* multiport, std::size_t size)
    : multiport_(multiport), desired_size_(size){
}

template<class T>
auto reactor::MutationChangeMultiportSize<T>::run() -> MutationResult {
  multiport_->ports_.resize(desired_size_);

  return Success;
}

template <class T>
auto reactor::MutationChangeMultiportSize<T>::rollback() -> MutationResult {
  return Success;
}


auto reactor::Transaction::execute() -> MutationResult {
  for (auto *mutation : mutations_) {
    mutation->run();
  }

  return Success;
}

void reactor::Transaction::push_back(reactor::Mutation* mutation) {
  mutations_.push_back(mutation);
}