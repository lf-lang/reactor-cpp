/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_MUTATIONS_BANK_HH
#define REACTOR_CPP_MUTATIONS_BANK_HH

#include <vector>

#include "../mutations.hh"
#include "../reactor.hh"

namespace reactor {
class Reactor;
class Environment;

template <class T> class MutationChangeBankSize final : public Mutation {
  std::vector<T>* bank_ = nullptr;
  std::size_t desired_size_ = 0;
  std::size_t size_before_application_ = 0;
  Reaction* reaction_ = nullptr;
  std::function<T(Environment* env, std::size_t index)> create_lambda_;

  void change_size(std::size_t new_size);

public:
  MutationChangeBankSize() = default;
  MutationChangeBankSize(const MutationChangeBankSize& other) noexcept
      : Mutation(other.reaction_)
      , bank_(other.bank_)
      , desired_size_(other.desired_size_)
      , size_before_application_(other.size_before_application_)
      , create_lambda_(other.create_lambda_) {}
  MutationChangeBankSize(MutationChangeBankSize&& other) noexcept
      : Mutation(other.reaction_)
      , bank_(other.bank_)
      , desired_size_(other.desired_size_)
      , size_before_application_(other.size_before_application_)
      , create_lambda_(other.create_lambda_) {}
  explicit MutationChangeBankSize(Reaction* reaction_, std::vector<T>* bank, std::size_t size,
                                  std::function<T(Environment* env, std::size_t index)> create_lambda);
  ~MutationChangeBankSize() override = default;
  auto operator=(const MutationChangeBankSize& other) -> MutationChangeBankSize& = default;
  auto operator=(MutationChangeBankSize&& other) -> MutationChangeBankSize& = default;

  auto run() -> MutationResult override;
  auto rollback() -> MutationResult override;
};
} // namespace reactor

#endif // REACTOR_CPP_MUTATIONS_BANK_HH
