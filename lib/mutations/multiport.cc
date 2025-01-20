//
// Created by tanneberger on 11/11/24.
//

#include "reactor-cpp/mutations/multiport.hh"
#include "reactor-cpp/reaction.hh"

template <class T>
reactor::MutationChangeOutputMultiportSize<T>::MutationChangeOutputMultiportSize(
    ModifableMultiport<Output<T>>* multiport, std::size_t size)
    : multiport_(multiport)
    , desired_size_(size) {}

template <class T> void reactor::MutationChangeOutputMultiportSize<T>::change_size(std::size_t new_size) {
  auto current_size = multiport_->size();
  if (current_size >= new_size) {
    // downscale
    multiport_->resize(new_size);

  } else {
    // upscale
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
