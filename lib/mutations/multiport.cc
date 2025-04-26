/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#include "reactor-cpp/mutations/multiport.hh"

template <class T>
reactor::MutationChangeOutputMultiportSize<T>::MutationChangeOutputMultiportSize(
    Reaction* reaction, ModifableMultiport<Output<T>>* multiport, const std::size_t size)
    : Mutation(reaction)
    , multiport_(multiport)
    , desired_size_(size) {}

template <class T> void reactor::MutationChangeOutputMultiportSize<T>::change_size(std::size_t new_size) {
  auto current_size = multiport_->size();
  if (current_size >= new_size) {
    // down-size
    multiport_->resize(new_size);

  } else {
    // up-size
    multiport_->reserve(new_size);
    for (auto i = 0; i < new_size - current_size; i++) {
      std::string port_name_ = multiport_->name() + "_" + std::to_string(current_size + i);
      multiport_->create_new_port();
      (*multiport_)[i + current_size].overwrite((*multiport_)[0]);
    }
  }
}
template <class T> auto reactor::MutationChangeOutputMultiportSize<T>::run() -> MutationResult {
  size_before_application_ = multiport_->size();
  change_size(desired_size_);
  return Success;
}

template <class T> auto reactor::MutationChangeOutputMultiportSize<T>::rollback() -> MutationResult {
  change_size(size_before_application_);
  return Success;
}
