//
// Created by tanneberger on 11/20/24.
//

#include "reactor-cpp/mutations/connection.hh"
#include "reactor-cpp/reactor.hh"

template <class A, class B>
reactor::MutationAddConnection<A, B>::MutationAddConnection(A* source, B* sink, Reactor* reactor) : source_(source), sink_(sink), reactor_(reactor) {

}


template <class A, class B> auto reactor::MutationAddConnection<A, B>::run() -> MutationResult {
  reactor_->environment()->draw_connection(source_, sink_, ConnectionProperties{});
  sink_->set_inward_binding(source_);
  source_->add_outward_binding(sink_);
  //std::cout << "from: " << source_->fqn() << "(" << source_ << ")"
  //                     << " --> to: " << sink_->fqn() << "(" << sink_ << ")" << std::endl;
  return Success;
}

template <class A, class B> auto reactor::MutationAddConnection<A, B>::rollback() -> MutationResult {
  reactor_->environment()->remove_connection(source_, sink_);

  return Success;
}


