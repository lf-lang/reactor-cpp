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

#include "multiport.hh"

namespace reactor {
class Reactor;
class Environment;

enum MutationResult {
  Success = 0,
  NotMatchingBankSize = 1,
};

class Mutation {
public:
  virtual auto run() -> MutationResult = 0 ;

};

template<class T>
class MutationChangeMultiportSize : public Mutation {
private:
  Multiport<T>* multiport_ = nullptr;
  std::size_t desired_size_ = 0;
public:
  MutationChangeMultiportSize(Multiport<T>* multiport, std::size_t size);
  ~MutationChangeMultiportSize() = default;
  void run() override;
};


class Transaction {
private:
  Reactor* parent = nullptr;
  Environment* environment = nullptr;
  std::vector<Mutation*> mutations_{};

public:
  void reset();
  auto execute() -> MutationResult;

};
}
#endif // REACTOR_CPP_TRANSACTION_HH
