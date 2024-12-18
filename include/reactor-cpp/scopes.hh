/*
 * Copyright (C) 2024 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#ifndef REACTOR_CPP_SCOPES_HH
#define REACTOR_CPP_SCOPES_HH

#include "environment.hh"
#include "logical_time.hh"
#include "reactor.hh"
#include "transaction.hh"

namespace reactor {

class Scope {
private:
  Reactor* reactor_;

public:
  explicit Scope(Reactor* reactor)
      : reactor_(reactor) {}

  [[nodiscard]] static auto get_physical_time() noexcept -> TimePoint { return Reactor::get_physical_time(); }
  [[nodiscard]] auto get_tag() const noexcept -> Tag { return reactor_->get_tag(); }
  [[nodiscard]] auto get_logical_time() const noexcept -> TimePoint { return reactor_->get_logical_time(); }
  [[nodiscard]] auto get_microstep() const noexcept -> mstep_t { return reactor_->get_microstep(); }
  [[nodiscard]] auto get_elapsed_logical_time() const noexcept -> Duration {
    return reactor_->get_elapsed_logical_time();
  }
  [[nodiscard]] auto get_elapsed_physical_time() const noexcept -> Duration {
    return reactor_->get_elapsed_physical_time();
  }
  [[nodiscard]] auto environment() const noexcept -> Environment* { return reactor_->environment(); }
  void request_stop() const { environment()->sync_shutdown(); }
};

class MutableScope : public Scope {
public:
  Transaction transaction_;
  Reactor* reactor_;
  Environment* env_ = nullptr;

  explicit MutableScope(Reactor* reactor)
      : Scope(reactor)
      , transaction_(reactor)
      , reactor_(reactor)
      , env_(reactor->environment()) {}
  MutableScope(const MutableScope& other)
      : Scope(other.reactor_)
      , transaction_(other.transaction_)
      , reactor_(other.reactor_)
      , env_(other.env_) {}
  MutableScope(MutableScope&& other) noexcept
      : Scope(other.reactor_)
      , transaction_(std::move(other.transaction_))
      , reactor_(other.reactor_)
      , env_(other.env_) {}
  ~MutableScope() = default;
  auto operator=(const MutableScope& other) -> MutableScope& = default;
  auto operator=(MutableScope&& other) -> MutableScope& = default;

  void commit_transaction(bool recalculate = false);
  void add_to_transaction(const std::shared_ptr<Mutation>& mutation);
};

} // namespace reactor

#endif // REACTOR_CPP_SCOPES_HH
