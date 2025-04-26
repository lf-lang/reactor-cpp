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
reactor::MutationAddConnection<A, B>::MutationAddConnection(Reaction* reaction, A* source, B* sink, bool add_connection)
    : Mutation(reaction)
    , source_(source)
    , sink_(sink)
    , add_connection_(add_connection) {}

template <class A, class B> auto reactor::MutationAddConnection<A, B>::run() -> MutationResult {
  auto* env = reaction_->environment();
  if (add_connection_) {
    env->draw_connection(source_, sink_, ConnectionProperties{});
  } else {
    env->remove_connection(source_, sink_);
  }

  return Success;
}

template <class A, class B> auto reactor::MutationAddConnection<A, B>::rollback() -> MutationResult {
  auto* env = reaction_->environment();
  if (add_connection_) {
    env->remove_connection(source_, sink_);
  } else {
    env->draw_connection(source_, sink_, ConnectionProperties{});
  }

  return Success;
}
