#include "reactor-cpp/dependency_graph.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/reactor.hh"

#include <boost/graph/directed_graph.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/property_maps/constant_property_map.hpp>
#include <boost/pending/property.hpp>
#include <boost/property_map/function_property_map.hpp>

#include <map>
#include <set>
#include <string>

using namespace boost;

namespace reactor {

struct reaction_info_t {
  using kind = vertex_property_tag;
};

using ReactionProperty = property<reaction_info_t, const Reaction*>;
using DependencyGraph = directed_graph<ReactionProperty>;
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

  property_map<DependencyGraph, reaction_info_t>::type reaction_proprty_map = get(reaction_info_t(), graph);
  for (auto entry : vertex_map) {
    put(reaction_proprty_map, entry.second, entry.first);
  }

  dynamic_properties dp;
  dp.property("node_id", get(boost::vertex_index, graph));
  dp.property("label", make_function_property_map<DependencyGraph::vertex_descriptor, std::string>(
                           [&reaction_proprty_map](DependencyGraph::vertex_descriptor vertex) {
                             return boost::get(reaction_proprty_map, vertex)->name();
                           }));
  dp.property("tooltip", make_function_property_map<DependencyGraph::vertex_descriptor, std::string>(
                             [&reaction_proprty_map](DependencyGraph::vertex_descriptor vertex) {
                               return "container: " + boost::get(reaction_proprty_map, vertex)->container()->fqn();
                             }));
  dp.property("fillcolor",
              make_function_property_map<DependencyGraph::vertex_descriptor, std::string>(
                  [&reaction_proprty_map](DependencyGraph::vertex_descriptor vertex) {
                    auto hash = std::hash<std::string>{}(boost::get(reaction_proprty_map, vertex)->container()->fqn());
                    auto red = (hash & 0xff0000) >> 16; // NOLINT
                    auto green = (hash & 0x00ff00) >> 9; // NOLINT
                    auto blue = (hash & 0x0000ff); // NOLINT
                    std::stringstream ss;
                    ss << "#" << std::setfill('0') << std::setw(2) << std::hex << red << green << blue;
                    return ss.str();
                  }));
  dp.property("style", make_constant_property<DependencyGraph::vertex_descriptor, std::string>("filled"));

  std::ofstream dot_file("test.dot");
  write_graphviz_dp(dot_file, graph, dp);
}

} // namespace reactor
