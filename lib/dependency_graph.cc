#include <fstream>

#include "reactor-cpp/dependency_graph.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/reactor.hh"
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/graphviz.hpp>

using namespace boost;

namespace reactor {

using DependencyGraph = directed_graph<>;
using ReactionToVertexMap = std::map<Reaction*, DependencyGraph::vertex_descriptor>;

void populate_graph_with_reactions(DependencyGraph& graph, ReactionToVertexMap& vertex_map, Reactor* reactor) {
  for (auto* reaction : reactor->reactions()) {
    vertex_map[reaction] = graph.add_vertex();
  }
  for (auto* sub_reactor : reactor->reactors()) {
    populate_graph_with_reactions(graph, vertex_map, sub_reactor);
  }
}

void generate_dependency_graph(std::set<Reactor*>& top_level_reactors) {

  DependencyGraph graph{};
  ReactionToVertexMap vertex_map{};

  for (auto* reactor : top_level_reactors) {
    populate_graph_with_reactions(graph, vertex_map, reactor);
  }

  std::ofstream dot_file("test.dot");
  write_graphviz(dot_file, graph);
}

} // namespace reactor
