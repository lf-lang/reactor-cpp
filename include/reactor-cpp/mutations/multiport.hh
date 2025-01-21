/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_MUTATIONS_MULTIPORT_HH
#define REACTOR_CPP_MUTATIONS_MULTIPORT_HH

#include <vector>

#include "../multiport.hh"
#include "../mutations.hh"
#include "../port.hh"

namespace reactor {
class Reactor;
class Environment;

template <class T> class MutationChangeOutputMultiportSize : public Mutation {
private:
  ModifableMultiport<Output<T>>* multiport_ = nullptr;
  std::size_t desired_size_ = 0;
  std::size_t size_before_application_ = 0;

  void change_size(std::size_t new_size);

public:
  MutationChangeOutputMultiportSize(ModifableMultiport<Output<T>>* multiport, std::size_t size);
  MutationChangeOutputMultiportSize() = default;
  MutationChangeOutputMultiportSize(const MutationChangeOutputMultiportSize& other)
      : multiport_(other.multiport_)
      , desired_size_(other.desired_size_)
      , size_before_application_(other.size_before_application_) {}
  MutationChangeOutputMultiportSize(MutationChangeOutputMultiportSize&& other) noexcept
      : multiport_(other.multiport_)
      , desired_size_(other.desired_size_)
      , size_before_application_(other.size_before_application_) {}
  ~MutationChangeOutputMultiportSize() override = default;
  auto operator=(const MutationChangeOutputMultiportSize& other) -> MutationChangeOutputMultiportSize& = default;
  auto operator=(MutationChangeOutputMultiportSize&& other) -> MutationChangeOutputMultiportSize& = default;

  auto run() -> MutationResult override;
  auto rollback() -> MutationResult override;
};
} // namespace reactor

#endif // REACTOR_CPP_MUTATIONS_MULTIPORT_HH
