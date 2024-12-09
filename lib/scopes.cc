

#include "reactor-cpp/scopes.hh"

void reactor::MutableScope::add_to_transaction(Mutation* mutation) {
  transaction_.push_back(mutation);
}

void reactor::MutableScope::commit_transaction() {
  transaction_.execute();
}