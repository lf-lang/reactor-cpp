/*
 * Copyright (C) 2019 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#include "reactor-cpp/transaction.hh"
#include "reactor-cpp/environment.hh"
#include "reactor-cpp/reactor.hh"

reactor::Transaction::Transaction(Environment* env)
    : environment_(env) {}

auto reactor::Transaction::execute(const bool recalculate) -> MutationResult {
  this->environment_->start_mutation();

  std::size_t index = 0;
  for (const auto& mutation : mutations_) {
    if (mutation->run() != Success) {
      break;
    }

    index++;
  }

  if (index != mutations_.size()) {
    for (std::size_t i = 0; i < index; i++) {
      mutations_[index - i]->rollback();
    }
  }

  if (recalculate) {
    this->environment_->calculate_indexes();
  }

  this->environment_->stop_mutation();

  mutations_.clear();
  return Success;
}

void reactor::Transaction::push_back(const std::shared_ptr<reactor::Mutation>& mutation) {
  mutations_.push_back(mutation);
}
