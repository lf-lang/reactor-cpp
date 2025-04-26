/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_MUTATIONS_CONNECTION_HH
#define REACTOR_CPP_MUTATIONS_CONNECTION_HH

#include "../mutations.hh"

namespace reactor {
class Reactor;
class Environment;

template <class A, class B> class MutationAddConnection : public Mutation {
private:
  A* source_;
  B* sink_;
  bool add_connection_ = true;
  bool connection_ = false;

public:
  explicit MutationAddConnection(Reaction* reaction, A* source, B* sink, bool add_connection);
  MutationAddConnection(const MutationAddConnection& other)
      : Mutation(other.reaction_)
      , source_(other.source_)
      , sink_(other.sink_)
      , add_connection_(other.add_connection_)
      , connection_(other.connection_) {}
  MutationAddConnection(MutationAddConnection&& other) noexcept
      : Mutation(other.reaction_)
      , source_(other.source_)
      , sink_(other.sink_)
      , connection_(other.connection_) {}
  MutationAddConnection() = default;
  ~MutationAddConnection() override = default;
  auto operator=(const MutationAddConnection& other) -> MutationAddConnection& = default;
  auto operator=(MutationAddConnection&& other) -> MutationAddConnection& = default;

  auto run() -> MutationResult override;
  auto rollback() -> MutationResult override;
};
} // namespace reactor

#endif // REACTOR_CPP_MUTATIONS_CONNECTION_HH
