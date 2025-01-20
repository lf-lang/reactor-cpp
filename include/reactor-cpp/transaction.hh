/*
 * Copyright (C) 2024 TU Dresden
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
// #include "reactor.hh"

namespace reactor {
class Reactor;
class Environment;

class Transaction {
private:
  Environment* environment_ = nullptr;
  Reactor* parent_ = nullptr;
  std::vector<std::shared_ptr<Mutation>> mutations_{};

public:
  explicit Transaction(Reactor* parent);
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
