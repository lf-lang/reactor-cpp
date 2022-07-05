#include "reactor-cpp/dependency_graph.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/reactor.hh"

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/graph_mutability_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/property_maps/constant_property_map.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/transitive_reduction.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/range/iterator_range_core.hpp>

#include <ctime>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>

using namespace boost;

namespace reactor {

void ReactionDependencyGraph::populate_graph_with_reactions(const Reactor* reactor) {
  for (auto* reaction : reactor->reactions()) {
    vertex_map[reaction] = graph.add_vertex();
  }
  for (auto* sub_reactor : reactor->reactors()) {
    populate_graph_with_reactions(sub_reactor);
  }
}

void ReactionDependencyGraph::populate_graph_with_priority_edges(const Reactor* reactor) {
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
    populate_graph_with_priority_edges(sub_reactor);
  }
}

void ReactionDependencyGraph::populate_graph_with_dependency_edges(const Reactor* reactor) {
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
    populate_graph_with_dependency_edges(sub_reactor);
  }
}

ReactionDependencyGraph::ReactionDependencyGraph(const std::set<Reactor*>& top_level_reactors) {
  for (auto* reactor : top_level_reactors) {
    populate_graph_with_reactions(reactor);
    populate_graph_with_priority_edges(reactor);
    populate_graph_with_dependency_edges(reactor);
  }

  // annotate edge vertex with a property that points to the reaction it corresponds to
  auto reaction_proprty_map = get_reaction_property_map();
  for (auto entry : vertex_map) {
    put(reaction_proprty_map, entry.second, entry.first);
  }
}

auto ReactionDependencyGraph::transitive_reduction() const -> ReactionDependencyGraph {
  ReactionDependencyGraph reduced{};

  // transitive_reduction uses this to populate a mapping from original vertices to new vertices in the reduced graph
  std::map<ReactionGraph::vertex_descriptor, ReactionGraph::vertex_descriptor> graph_to_reduced_graph{};
  // transitive reduction needs this mapping of vertices to integers
  std::map<ReactionGraph::vertex_descriptor, std::size_t> id_map{};
  size_t id{0};
  for (ReactionGraph::vertex_descriptor vd : boost::make_iterator_range(vertices(graph))) {
    id_map[vd] = id++;
  }

  // perform the actual reduction
  boost::transitive_reduction(graph, reduced.graph, make_assoc_property_map(graph_to_reduced_graph),
                              make_assoc_property_map(id_map));

  // update the mapping of reactions to vertices
  auto reaction_property_map = reduced.get_reaction_property_map();
  for (ReactionGraph::vertex_descriptor vd : boost::make_iterator_range(vertices(graph))) {
    put(reaction_property_map, graph_to_reduced_graph[vd], get(reaction_property_map, vd));
  }

  return reduced;
}

void ReactionDependencyGraph::export_graphviz(const std::string& file_name) {
  auto reaction_proprty_map = get_reaction_property_map();
  dynamic_properties dp;
  dp.property("node_id", get(boost::vertex_index, graph));
  dp.property("label", make_function_property_map<ReactionGraph::vertex_descriptor, std::string>(
                           [&reaction_proprty_map](ReactionGraph::vertex_descriptor vertex) {
                             return boost::get(reaction_proprty_map, vertex)->name();
                           }));
  dp.property("tooltip", make_function_property_map<ReactionGraph::vertex_descriptor, std::string>(
                             [&reaction_proprty_map](ReactionGraph::vertex_descriptor vertex) {
                               return "container: " + boost::get(reaction_proprty_map, vertex)->container()->fqn();
                             }));
  dp.property("fillcolor",
              make_function_property_map<ReactionGraph::vertex_descriptor, std::string>(
                  [&reaction_proprty_map](ReactionGraph::vertex_descriptor vertex) {
                    auto hash = std::hash<std::string>{}(boost::get(reaction_proprty_map, vertex)->container()->fqn());
                    auto red = (hash & 0xff0000) >> 16;  // NOLINT
                    auto green = (hash & 0x00ff00) >> 9; // NOLINT
                    auto blue = (hash & 0x0000ff);       // NOLINT
                    std::stringstream ss;
                    ss << "#" << std::setfill('0') << std::setw(2) << std::hex << red << green << blue;
                    return ss.str();
                  }));
  dp.property("style", make_constant_property<ReactionGraph::vertex_descriptor, std::string>("filled"));

  std::ofstream dot_file(file_name);
  write_graphviz_dp(dot_file, graph, dp);
}

GroupedDependencyGraph::GroupedDependencyGraph(ReactionDependencyGraph& reactionGraph) {
  // the lambda below also has the side-effect of updating vertex map
  copy_graph(reactionGraph.graph, graph,
             vertex_copy([this, &reactionGraph](ReactionDependencyGraph::ReactionGraph::vertex_descriptor in,
                                                GroupGraph::vertex_descriptor out) {
               Group group{};
               group.push_back(boost::get(reactionGraph.get_reaction_property_map(), in));
               boost::put(get_group_property_map(), out, group);
               vertex_map[group[0]] = out;
             }));
}

void GroupedDependencyGraph::export_graphviz(const std::string& file_name) {
  auto group_proprty_map = get_group_property_map();
  dynamic_properties dp;
  dp.property("node_id", get(boost::vertex_index, graph));
  dp.property("label", make_function_property_map<GroupGraph::vertex_descriptor, std::string>(
                           [&group_proprty_map](GroupGraph::vertex_descriptor vertex) {
                             std::stringstream ss;
                             for (const auto* reaction : boost::get(group_proprty_map, vertex)) {
                               auto sep = ss.tellp() == 0 ? '{' : ',';
                               ss << sep << reaction->name();
                             }
                             ss << '}';
                             return ss.str();
                           }));
  dp.property("tooltip", make_function_property_map<GroupGraph::vertex_descriptor, std::string>(
                             [&group_proprty_map](GroupGraph::vertex_descriptor vertex) {
                               std::stringstream ss;
                               for (const auto* reaction : boost::get(group_proprty_map, vertex)) {
                                 ss << "reactions: \n";
                                 ss << "  - " << reaction->fqn();
                               }
                               ss << '}';
                               return ss.str();
                             }));
  dp.property("fillcolor",
              make_function_property_map<GroupGraph::vertex_descriptor, std::string>(
                  [&group_proprty_map](GroupGraph::vertex_descriptor vertex) {
                    auto hash = std::hash<std::string>{}(boost::get(group_proprty_map, vertex)[0]->container()->fqn());
                    auto red = (hash & 0xff0000) >> 16;  // NOLINT
                    auto green = (hash & 0x00ff00) >> 9; // NOLINT
                    auto blue = (hash & 0x0000ff);       // NOLINT
                    std::stringstream ss;
                    ss << "#" << std::setfill('0') << std::setw(2) << std::hex << red << green << blue;
                    return ss.str();
                  }));
  dp.property("style", make_constant_property<GroupGraph::vertex_descriptor, std::string>("filled"));

  std::ofstream dot_file(file_name);
  write_graphviz_dp(dot_file, graph, dp);
}

void GroupedDependencyGraph::try_contract_edge(const Reaction* a, const Reaction* b) {
  GroupGraph::vertex_descriptor va = vertex_map[a];
  GroupGraph::vertex_descriptor vb = vertex_map[b];

  // if both vertexes are the same we can abort...
  if (va == vb) {
    return;
  }

  // we remove the edge that we want to contract
  remove_edge(va, vb, graph);
  // and then check if there is another path from a to b
  if (has_path(a, b)) {
    // If there is another path from a to b, contracting would introduce a cycle into the graph. Thus we abort here and
    // simply add the edge back that we removed earlier.
    add_edge(va, vb, graph);
  } else {
    // it is safe to contract.

    // route all edges in/out of vb to va
    for (auto edge : make_iterator_range(out_edges(vb, graph))) {
      add_edge(va, target(edge, graph), graph);
    }
    for (auto edge : make_iterator_range(in_edges(vb, graph))) {
      add_edge(source(edge, graph), va, graph);
    }

    // update va's properties
    auto& va_reactions = boost::get(get_group_property_map(), va);
    auto& vb_reactions = boost::get(get_group_property_map(), vb);
    va_reactions.insert(va_reactions.end(), vb_reactions.begin(), vb_reactions.end());

    // update the vertex mapping
    for (const auto* reaction : vb_reactions) {
      vertex_map[reaction] = va;
    }

    // delete vb
    clear_vertex(vb, graph);
    remove_vertex(vb, graph);
  }
}

auto GroupedDependencyGraph::has_path(const Reaction* a, const Reaction* b) const -> bool {
  GroupGraph::vertex_descriptor va = vertex_map.at(a);
  GroupGraph::vertex_descriptor vb = vertex_map.at(b);

  try {
    // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDelete) [prevents a warning triggered from the boost headers]
    breadth_first_search(graph, va, visitor(ReachabilityVisitor(vb)));
  } catch (const ReachabilityVisitor::PathExists& e) {
    return true;
  }
  return false;
}

void GroupedDependencyGraph::group_reactions_by_container(const std::set<Reactor*>& top_level_reactors) {
  for (const auto* reactor : top_level_reactors) {
    group_reactions_by_container_helper(reactor);
  }
}

void GroupedDependencyGraph::group_reactions_by_container_helper(const Reactor* reactor) {
  const auto& reactions = reactor->reactions();

  if (reactions.size() > 1) {
    auto it = reactions.begin();
    auto next = std::next(it);
    while (next != reactions.end()) {
      try_contract_edge(*it, *next);
      it = next;
      next = std::next(it);
    }
  }

  for (const auto* r : reactor->reactors()) {
    group_reactions_by_container_helper(r);
  }
}
} // namespace reactor
