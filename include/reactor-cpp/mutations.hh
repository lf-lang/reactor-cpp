/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_MUTATIONS_HH
#define REACTOR_CPP_MUTATIONS_HH

#include "reaction.hh"

#include <cinttypes>

namespace reactor {
class Reactor;
class Environment;

enum MutationResult : std::int8_t {
  Success = 0,
  NotMatchingBankSize = 1,
};

class Mutation {
protected:
  Reaction* reaction_ = nullptr;

public:
  Mutation(Reaction* reaction)
      : reaction_(reaction) {};
  Mutation(const Mutation& other)
      : reaction_(other.reaction_) {};
  Mutation(Mutation&& other) noexcept
      : reaction_(other.reaction_) {};
  virtual ~Mutation() = default;
  auto operator=(const Mutation& other) -> Mutation& = default;
  auto operator=(Mutation&& other) -> Mutation& = default;

  virtual auto run() -> MutationResult = 0;
  virtual auto rollback() -> MutationResult = 0;
  virtual auto level() -> std::size_t { return reaction_->index(); };
};

} // namespace reactor

#endif // REACTOR_CPP_MUTATIONS_HH
