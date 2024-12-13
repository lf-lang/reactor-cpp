/*
* Copyright (C) 2024 TU Dresden
* All rights reserved.
*
* Authors:
*   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_TRANSACTION_HH
#define REACTOR_CPP_TRANSACTION_HH

#include <vector>
#include <memory>

#include "mutations.hh"
//#include "reactor.hh"

namespace reactor {
class Reactor;
class Environment;


class Transaction {
private:
  Environment* environment_ = nullptr;
  std::vector<std::shared_ptr<Mutation>> mutations_{};

public:
  explicit Transaction(Reactor* parent);
  ~Transaction() = default;

  void push_back(const std::shared_ptr<Mutation>& mutation);
  auto execute(bool recalculate = false) -> MutationResult;
};
}
#endif // REACTOR_CPP_TRANSACTION_HH
