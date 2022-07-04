#include <fstream>

#include "reactor-cpp/dependency_graph.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/reactor.hh"
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/graphviz.hpp>

using namespace boost;

namespace reactor {

using DependencyGraph = directed_graph<>;
using ReactionToVertexMap = std::map<const Reaction*, DependencyGraph::vertex_descriptor>;

void populate_graph_with_reactions(DependencyGraph& graph, ReactionToVertexMap& vertex_map, const Reactor* reactor) {
  for (auto* reaction : reactor->reactions()) {
    vertex_map[reaction] = graph.add_vertex();
  }
  for (auto* sub_reactor : reactor->reactors()) {
    populate_graph_with_reactions(graph, vertex_map, sub_reactor);
  }
}

void populate_graph_with_priority_edges(DependencyGraph& graph, const ReactionToVertexMap& vertex_map,
                                        const Reactor* reactor) {
  const auto& reactions = reactor->reactions();

  if (reactions.size() > 1) {
    auto iterator = reactions.begin();
    auto next = std::next(iterator);
    while (next != reactions.end()) {
      graph.add_edge(vertex_map.at(*iterator), vertex_map.at(*next));
      iterator = next;
      next = std::next(iterator);
    }
  }

  for (auto* sub_reactor : reactor->reactors()) {
    populate_graph_with_priority_edges(graph, vertex_map, sub_reactor);
  }
}

void populate_graph_with_dependency_edges(DependencyGraph& graph, const ReactionToVertexMap& vertex_map,
                                          const Reactor* reactor) {
  for (auto* reaction : reactor->reactions()) {
    for (auto* dependency : reaction->dependencies()) {
      auto* source = dependency;
      while (source->has_inward_binding()) {
        source = source->inward_binding();
      }
      for (auto* antidependency : source->antidependencies()) {
        graph.add_edge(vertex_map.at(antidependency), vertex_map.at(reaction));
      }
    }
  }

  for (auto* sub_reactor : reactor->reactors()) {
    populate_graph_with_dependency_edges(graph, vertex_map, sub_reactor);
  }
}

void generate_dependency_graph(std::set<Reactor*>& top_level_reactors) {

  DependencyGraph graph{};
  ReactionToVertexMap vertex_map{};

  for (auto* reactor : top_level_reactors) {
    populate_graph_with_reactions(graph, vertex_map, reactor);
    populate_graph_with_priority_edges(graph, vertex_map, reactor);
    populate_graph_with_dependency_edges(graph, vertex_map, reactor);
  }

  std::ofstream dot_file("test.dot");
  write_graphviz(dot_file, graph);
}

} // namespace reactor
