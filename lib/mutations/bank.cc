//
// Created by tanneberger on 11/11/24.
//

#include "reactor-cpp/mutations/bank.hh"
#include "reactor-cpp/action.hh"

template <class T>
reactor::MutationChangeBankSize<T>::MutationChangeBankSize(
    std::vector<T>* bank, Environment* env, std::size_t size,
    std::function<T(Environment* env, std::size_t index)> create_lambda)
    : bank_(bank)
    , env_(env)
    , desired_size_(size)
    , create_lambda_(std::move(create_lambda)) {}

template <class T> void reactor::MutationChangeBankSize<T>::change_size(std::size_t new_size) {
  auto current_size = bank_->size();

  if (current_size >= new_size) { // downscale
    bank_->resize(new_size);

  } else { // upscale
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
