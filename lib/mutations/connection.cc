/*
 * Copyright (C) 2025 TU Dresden
 * All rights reserved.
 *
 * Authors:
 *   Tassilo Tanneberger
 */

#include "reactor-cpp/mutations/connection.hh"
#include "reactor-cpp/reactor.hh"

template <class A, class B>
reactor::MutationAddConnection<A, B>::MutationAddConnection(A* source, B* sink, Environment* env, bool add_connection)
    : source_(source)
    , sink_(sink)
    , add_connection_(add_connection)
    , env_(env) {}

template <class A, class B> auto reactor::MutationAddConnection<A, B>::run() -> MutationResult {
  if (add_connection_) {
    env_->draw_connection(source_, sink_, ConnectionProperties{});
  } else {
    env_->remove_connection(source_, sink_);
  }

  return Success;
}

template <class A, class B> auto reactor::MutationAddConnection<A, B>::rollback() -> MutationResult {
  if (add_connection_) {
    env_->remove_connection(source_, sink_);
  } else {
    env_->draw_connection(source_, sink_, ConnectionProperties{});
  }

  return Success;
}
