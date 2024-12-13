

#include "reactor-cpp/scopes.hh"

void reactor::MutableScope::add_to_transaction(const std::shared_ptr<Mutation>& mutation) {
  transaction_.push_back(mutation);
}

void reactor::MutableScope::commit_transaction(bool recalculate) {
  (void)recalculate;
  transaction_.execute(recalculate);
}