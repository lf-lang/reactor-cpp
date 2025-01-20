//
// Created by tanneberger on 11/18/24.
//

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
  Environment* env_ = nullptr;

public:
  explicit MutationAddConnection(A* source, B* sink, Environment* env, bool add_connection);
  MutationAddConnection(const MutationAddConnection& other)
      : source_(other.source_)
      , sink_(other.sink_)
      , add_connection_(other.add_connection_)
      , connection_(other.connection_)
      , env_(other.env_) {}
  MutationAddConnection(MutationAddConnection&& other) noexcept
      : source_(other.source_)
      , sink_(other.sink_)
      , connection_(other.connection_)
      , env_(other.env_) {}
  MutationAddConnection() = default;
  ~MutationAddConnection() override = default;
  auto operator=(const MutationAddConnection& other) -> MutationAddConnection& = default;
  auto operator=(MutationAddConnection&& other) -> MutationAddConnection& = default;

  auto run() -> MutationResult override;
  auto rollback() -> MutationResult override;
};
} // namespace reactor

#endif // REACTOR_CPP_MUTATIONS_CONNECTION_HH
