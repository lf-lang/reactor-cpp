/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#include "reactor-cpp/mutations/bank.hh"
#include "reactor-cpp/action.hh"

template <class T>
reactor::MutationChangeBankSize<T>::MutationChangeBankSize(
    std::vector<T>* bank, Environment* env, const std::size_t size,
    std::function<T(Environment* env, std::size_t index)> create_lambda)
    : bank_(bank)
    , desired_size_(size)
    , env_(env)
    , create_lambda_(std::move(create_lambda)) {}

template <class T> void reactor::MutationChangeBankSize<T>::change_size(std::size_t new_size) {
  auto current_size = bank_->size();

  if (current_size >= new_size) { // down-size
    bank_->resize(new_size);

  } else { // up-size
    bank_->reserve(new_size);

    for (auto i = 0; i < new_size - current_size; i++) {
      bank_->push_back(create_lambda_(env_, current_size + i));
      (*bank_)[bank_->size() - 1]->assemble();
    }
  }
}
template <class T> auto reactor::MutationChangeBankSize<T>::run() -> MutationResult {
  size_before_application_ = bank_->size();
  change_size(desired_size_);
  return Success;
}

template <class T> auto reactor::MutationChangeBankSize<T>::rollback() -> MutationResult {
  change_size(size_before_application_);
  return Success;
}
