//
// Created by tanneberger on 11/11/24.
//

#include "reactor-cpp/mutations/multiport.hh"
#include "reactor-cpp/reaction.hh"

template <class T>
reactor::MutationChangeOutputMultiportSize<T>::MutationChangeOutputMultiportSize(
    ModifableMultiport<Output<T>>* multiport, Reactor* reactor, std::set<Reaction*>& anti_dependencies,
    std::size_t size)
    : multiport_(multiport)
    , reactor_(reactor)
    , desired_size_(size)
    , anti_dependencies_(anti_dependencies) {}

template <class T> void reactor::MutationChangeOutputMultiportSize<T>::change_size(std::size_t new_size) {
  auto current_size = multiport_->size();
  if (current_size >= new_size) {
    // downscale

    for (auto i = 0; i < current_size - new_size; i++) {
      // TODO: consider saving the ports here here

      std::string port_name_ = multiport_->name() + "_" + std::to_string(current_size - i - 1);
      multiport_->pop_back();
      auto base_port = Output<T>{port_name_, reactor_};
      reactor_->remove_inputs(&base_port);
    }

    for (auto* anti_dep : anti_dependencies_) {
      anti_dep->clear_antidependencies();
      for (auto i = 0; i < new_size; i++) {

        multiport_->operator[](i).anti_dependencies().clear();
        anti_dep->declare_antidependency(&multiport_->operator[](i));
      }
    }
  } else {
    // upscale

    for (auto i = 0; i < new_size - current_size; i++) {
      std::string port_name_ = multiport_->name() + "_" + std::to_string(current_size + i);
      multiport_->emplace_back(port_name_, reactor_);
    }

    /*for (auto* anti_dep : anti_dependencies_) {
        anti_dep->clear_antidependencies();
        for (auto i = 0; i < new_size; i++) {
            anti_dep->declare_antidependency(&multiport_->operator[](i));
        }
    }*/
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
