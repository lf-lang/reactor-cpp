/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_TRANSACTION_HH
#define REACTOR_CPP_TRANSACTION_HH

#include <memory>
#include <vector>

#include "mutations.hh"

namespace reactor {
class Reactor;
class Environment;

class Transaction {
  Environment* environment_ = nullptr;
  std::vector<std::shared_ptr<Mutation>> mutations_{};

public:
  explicit Transaction(Environment* env);
  Transaction(const Transaction& other) = default;
  Transaction(Transaction&& other) = default;
  auto operator=(const Transaction& other) -> Transaction& = default;
  auto operator=(Transaction&& other) -> Transaction& = default;
  ~Transaction() = default;

  void push_back(const std::shared_ptr<Mutation>& mutation);
  auto execute(bool recalculate = false) -> MutationResult;
};
} // namespace reactor
#endif // REACTOR_CPP_TRANSACTION_HH
