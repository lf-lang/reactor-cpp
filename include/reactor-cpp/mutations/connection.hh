//
// Created by tanneberger on 11/18/24.
//

#ifndef MUTATION_CONNECTION_HH
#define MUTATION_CONNECTION_HH

#include "../mutations.hh"

namespace reactor {
class Reactor;
class Environment;

template<class A, class B>
class MutationAddConnection : public Mutation {
private:
  A* source_;
  B* sink_;
  bool connection_ = false;
  Reactor* reactor_{};

public:
  MutationAddConnection(A* source, B* sink, Reactor* reactor);
  MutationAddConnection(const MutationAddConnection& other) : source_(other.source_), sink_(other.sink_), connection_(other.connection_), reactor_(other.reactor_) {}
  MutationAddConnection() = default;
  ~MutationAddConnection() override = default;

  auto run() -> MutationResult override;
  auto rollback() -> MutationResult override;
};
}

#endif //MUTATION_CONNECTION_HH
