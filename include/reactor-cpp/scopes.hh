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
#include "logical_time.hh"
#include "reactor.hh"
#include "environment.hh"

namespace reactor {

class Scope {
private:
  Reactor* reactor;

public:
  Scope(Reactor* reactor)
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

class MutableScope : public Scope {
public:
  Transaction transaction_;
  Reactor* reactor_;
  Environment* env_ = nullptr;

  explicit MutableScope(Reactor* reactor) : Scope(reactor), transaction_(reactor), reactor_(reactor), env_(reactor->environment()) {}
  ~MutableScope() = default;

  void commit_transaction(bool recalculate = false);
  void add_to_transaction(const std::shared_ptr<Mutation>& mutation);

};

}

#endif // REACTOR_CPP_SCOPS_HH
