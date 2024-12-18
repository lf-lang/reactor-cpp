//
// Created by tanneberger on 11/11/24.
//

#include "reactor-cpp/mutations/bank.hh"
#include "reactor-cpp/action.hh"

template <class T>
reactor::MutationChangeBankSize<T>::MutationChangeBankSize(
    std::vector<T>* bank, Reactor* reactor, std::size_t size,
    std::function<T(Reactor* parent_reactor, std::size_t index)> create_lambda)
    : bank_(bank)
    , reactor_(reactor)
    , desired_size_(size)
    , create_lambda_(std::move(create_lambda)) {}

template <class T> void reactor::MutationChangeBankSize<T>::change_size(std::size_t new_size) {
  bank_->reserve(new_size);
  auto current_size = bank_->size();
  std::cout << "scaling from: " << current_size << " to " << new_size << std::endl;

  if (current_size >= new_size) {
    // downscale

    for (auto i = 0; i < current_size - new_size; i++) {
      // TODO: consider saving the ports here here
      std::unique_ptr<Reactor> lastElement = std::move(bank_->back());
      bank_->pop_back();
      reactor_->environment()->remove_top_level_reactor(lastElement.get());
    }
  } else {
    // upscale

    for (auto i = 0; i < new_size - current_size; i++) {
      bank_->push_back(create_lambda_(reactor_, current_size + i));
      (*bank_)[bank_->size() - 1]->assemble();
    }
    std::cout << "created new reactors" << '\n';
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
