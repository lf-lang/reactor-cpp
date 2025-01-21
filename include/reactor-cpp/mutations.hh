/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_MUTATIONS_HH
#define REACTOR_CPP_MUTATIONS_HH

#include <cinttypes>

namespace reactor {
class Reactor;
class Environment;

enum MutationResult : std::int8_t {
  Success = 0,
  NotMatchingBankSize = 1,
};

class Mutation {
public:
  Mutation() = default;
  Mutation(const Mutation& other) = default;
  Mutation(Mutation&& other) = default;
  virtual ~Mutation() = default;
  auto operator=(const Mutation& other) -> Mutation& = default;
  auto operator=(Mutation&& other) -> Mutation& = default;

  virtual auto run() -> MutationResult = 0;
  virtual auto rollback() -> MutationResult = 0;
};

} // namespace reactor

#endif // REACTOR_CPP_MUTATIONS_HH
