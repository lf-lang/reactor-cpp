/*
* Copyright (C) 2024 TU Dresden
* All rights reserved.
*
* Authors:
*   Tassilo Tanneberger
*/

#ifndef REACTOR_CPP_SCOPS_HH
#define REACTOR_CPP_SCOPS_HH

#include "transaction.hh"
#include "time.hh"

namespace reactor {
class Reactor;

class Scope {
private:
  reactor::Reactor* reactor;

public:
  Scope(reactor::Reactor* reactor)
      : reactor(reactor) {}

  auto get_physical_time() const -> reactor::TimePoint { return reactor->get_physical_time(); }
  auto get_tag() const -> reactor::Tag { return reactor->get_tag(); }
  auto get_logical_time() const -> reactor::TimePoint { return reactor->get_logical_time(); }
  auto get_microstep() const -> reactor::mstep_t { return reactor->get_microstep(); }
  auto get_elapsed_logical_time() const -> reactor::Duration { return reactor->get_elapsed_logical_time(); }
  auto get_elapsed_physical_time() const -> reactor::Duration { return reactor->get_elapsed_physical_time(); }
  auto environment() const -> reactor::Environment* { return reactor->environment(); }
  void request_stop() const { return environment()->sync_shutdown(); }
};

template<class HostReactor>
class MutableScope : public Scope {
public:
  HostReactor* self_ = nullptr;
  Environment* env_ = nullptr;
  Transaction transaction_;

  explicit MutableScope(reactor::Reactor* reactor) : Scope(reactor), self_(reactor), env_(reactor->environment()) {}
  ~MutableScope() = default;

  void commit_transaction();
  void add_to_transaction(Mutation* mutation);

};

}

#endif // REACTOR_CPP_SCOPS_HH
