#ifndef REACTOR_CPP_DEPENDENCY_GRAPH_HH
#define REACTOR_CPP_DEPENDENCY_GRAPH_HH

#include "reactor-cpp/reactor.hh"

namespace reactor {

void generate_dependency_graph(std::set<Reactor*>& top_level_reactors);

} // namespace reactor

#endif
