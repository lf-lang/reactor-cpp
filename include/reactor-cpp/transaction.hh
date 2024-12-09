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

#include "mutations.hh"
//#include "reactor.hh"

namespace reactor {
class Reactor;
class Environment;


class Transaction {
private:
  Reactor* parent_ = nullptr;
  Environment* environment_ = nullptr;
  std::vector<Mutation*> mutations_{};

public:
  explicit Transaction(Reactor* parent);
  ~Transaction() = default;

  void push_back(Mutation* mutation);
  auto execute() -> MutationResult;
};
}
#endif // REACTOR_CPP_TRANSACTION_HH
