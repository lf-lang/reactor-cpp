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
protected:
  Reaction* reaction_ = nullptr;

public:
  explicit Scope(Reaction* reaction)
      : reaction_(reaction) {}

  [[nodiscard]] static auto get_physical_time() noexcept -> TimePoint { return Reactor::get_physical_time(); }
  [[nodiscard]] auto get_tag() const noexcept -> Tag { return reaction_->container()->get_tag(); }
  [[nodiscard]] auto get_logical_time() const noexcept -> TimePoint {
    return reaction_->container()->get_logical_time();
  }
  [[nodiscard]] auto get_microstep() const noexcept -> mstep_t { return reaction_->container()->get_microstep(); }
  [[nodiscard]] auto get_elapsed_logical_time() const noexcept -> Duration {
    return reaction_->container()->get_elapsed_logical_time();
  }
  [[nodiscard]] auto get_elapsed_physical_time() const noexcept -> Duration {
    return reaction_->container()->get_elapsed_physical_time();
  }
  [[nodiscard]] auto environment() const noexcept -> Environment* { return reaction_->container()->environment(); }
  void request_stop() const { environment()->sync_shutdown(); }
};

class MutableScope : public Scope {
public:
  Transaction transaction_;
  Environment* env_ = nullptr;

  explicit MutableScope(Reaction* reaction)
      : Scope(reaction)
      , transaction_(reaction->environment())
      , env_(reaction->environment()) {}
  MutableScope(const MutableScope& other)
      : Scope(other.reaction_)
      , transaction_(other.transaction_)
      , env_(other.env_) {}
  MutableScope(MutableScope&& other) noexcept
      : Scope(other.reaction_)
      , transaction_(std::move(other.transaction_))
      , env_(other.env_) {}
  ~MutableScope() = default;
  auto operator=(const MutableScope& other) -> MutableScope& = default;
  auto operator=(MutableScope&& other) -> MutableScope& = default;

  void commit_transaction(bool recalculate = false);
  void add_to_transaction(const std::shared_ptr<Mutation>& mutation);
};

} // namespace reactor

#endif // REACTOR_CPP_SCOPES_HH
