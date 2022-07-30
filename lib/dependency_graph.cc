#include "reactor-cpp/dependency_graph.hh"
#include "reactor-cpp/port.hh"
#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/reactor.hh"

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_mutability_traits.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/property_maps/constant_property_map.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/transitive_reduction.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/range/iterator_range_core.hpp>

#include <cstddef>
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
      auto edge = graph.add_edge(vertex_map.at(*iterator), vertex_map.at(*next)).first;
      put(get_dependency_property_map(), edge, DependencyType::Priority);
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
        auto edge = graph.add_edge(vertex_map.at(antidependency), vertex_map.at(reaction)).first;
        if (reaction->port_triggers().contains(dependency)) {
          put(get_dependency_property_map(), edge, DependencyType::Effect);
        } else {
          put(get_dependency_property_map(), edge, DependencyType::Trigger);
        }
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

auto ReactionDependencyGraph::transitive_reduction() -> ReactionDependencyGraph {
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
  for (ReactionGraph::vertex_descriptor vd : boost::make_iterator_range(vertices(graph))) {
    put(reduced.get_reaction_property_map(), graph_to_reduced_graph[vd], get(get_reaction_property_map(), vd));
  }

  return reduced;
}

void ReactionDependencyGraph::export_graphviz(const std::string& file_name) {
  auto reaction_proprty_map = get_reaction_property_map();
  auto dependency_proprty_map = get_dependency_property_map();
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
  dp.property("style", make_function_property_map<ReactionGraph::edge_descriptor, std::string>(
                           [&dependency_proprty_map](ReactionGraph::edge_descriptor edge) {
                             auto dependency_type = boost::get(dependency_proprty_map, edge);
                             switch (dependency_type) {
                             case DependencyType::Undefined:
                               return "solid";
                             case DependencyType::Priority:
                               return "dotted";
                             case DependencyType::Effect:
                               return "dashed";
                             case DependencyType::Trigger:
                               return "bold";
                             }
                             return "invis";
                           }));

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
             })
                 .edge_copy([]([[maybe_unused]] ReactionDependencyGraph::ReactionGraph::edge_descriptor in,
                               [[maybe_unused]] GroupGraph::edge_descriptor out) {
                   // do nothing; simply drop the edge descriptors
                 }));
}

void GroupedDependencyGraph::export_graphviz(const std::string& file_name) {
  auto group_proprty_map = get_group_property_map();
  dynamic_properties dp;
  dp.property("node_id", get(boost::vertex_index, graph));
  dp.property("label", make_function_property_map<GroupGraph::vertex_descriptor, std::string>(
                           [&group_proprty_map](GroupGraph::vertex_descriptor vertex) {
                             std::stringstream ss;
                             std::size_t count{0};
                             const auto& reactions = boost::get(group_proprty_map, vertex);

                             ss << '{';
                             for (const auto* reaction : reactions) {
                               count++;
                               ss << reaction->name();
                               if (count != reactions.size()) {
                                 ss << ',';
                                 if (count % 4 == 0) {
                                   ss << '\n';
                                 }
                               }
                             }
                             ss << '}';
                             return ss.str();
                           }));
  dp.property("tooltip", make_function_property_map<GroupGraph::vertex_descriptor, std::string>(
                             [&group_proprty_map](GroupGraph::vertex_descriptor vertex) {
                               std::stringstream ss;
                               ss << "reactions: \n";
                               for (const auto* reaction : boost::get(group_proprty_map, vertex)) {
                                 ss << "  - " << reaction->fqn() << '\n';
                               }
                               ss << '}';
                               return ss.str();
                             }));
  dp.property("fillcolor", make_function_property_map<GroupGraph::vertex_descriptor, std::string>(
                               [&group_proprty_map](GroupGraph::vertex_descriptor vertex) {
                                 auto& reactions = boost::get(group_proprty_map, vertex);
                                 if (!reactions.empty()) {
                                   auto hash = std::hash<std::string>{}(reactions[0]->container()->fqn());
                                   auto red = (hash & 0xff0000) >> 16;  // NOLINT
                                   auto green = (hash & 0x00ff00) >> 9; // NOLINT
                                   auto blue = (hash & 0x0000ff);       // NOLINT
                                   std::stringstream ss;
                                   ss << "#" << std::setfill('0') << std::setw(2) << std::hex << red << green << blue;
                                   return ss.str();
                                 }
                                 return std::string{"#ffffff"};
                               }));
  dp.property("style", make_constant_property<GroupGraph::vertex_descriptor, std::string>("filled"));

  std::ofstream dot_file(file_name);
  write_graphviz_dp(dot_file, graph, dp);
}

void GroupedDependencyGraph::try_contract_edge(GroupGraph::vertex_descriptor va, GroupGraph::vertex_descriptor vb) {
  // if both vertexes are the same we can abort...
  if (va == vb) {
    return;
  }

  // Abort if there is no direct edge between the vertexes
  if (!edge(va, vb, graph).second) {
    return;
  }

  // remove the direct edge between va and vb
  remove_edge(va, vb, graph);

  // check if there is still a path from va to vb; abort in this case as we would introduce a cycle when contracting the
  // edge
  if (has_path(va, vb)) {
    // add the original edge back and abort
    add_edge(va, vb, graph);
    return;
  }

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
  for (auto* reaction : vb_reactions) {
    vertex_map[reaction] = va;
  }

  // and remove all reactions listed in vb making it empty
  vb_reactions.clear();

  // remove all the edges from/to vb, but don't remove vb itself yet. Removing it would invalidate all the vertex
  // descriptors and iterators. Its better to later remove all empty vertices in a single go.
  clear_vertex(vb, graph);
}

void GroupedDependencyGraph::group_reactions_by_container(const std::set<Reactor*>& top_level_reactors) {
  for (const auto* reactor : top_level_reactors) {
    group_reactions_by_container_helper(reactor);
  }

  clear_all_empty_vertices();
}

void GroupedDependencyGraph::clear_all_empty_vertices() {
  // First, get a filtered view of the graph only showing the non-empty vertices
  NonEmptyGoupFilter filter{get_group_property_map()};
  filtered_graph<GroupGraph, keep_all, NonEmptyGoupFilter> filtered_graph{graph, keep_all(), filter};

  // Create a new graph and copy the contents from the filtered view.
  GroupGraph new_graph{};
  copy_graph(filtered_graph, new_graph);

  // Replace our current internal graph and clear the old one
  graph.swap(new_graph);
  new_graph.clear();

  // update the vertex map with the new vertex descriptors
  vertex_map.clear();
  for (GroupGraph::vertex_descriptor vd : boost::make_iterator_range(vertices(graph))) {
    const auto& reactions = get(get_group_property_map(), vd);
    for (auto* reaction : reactions) {
      vertex_map[reaction] = vd;
    }
  }
}

void GroupedDependencyGraph::group_reactions_by_container_helper(const Reactor* reactor) {
  const auto& reactions = reactor->reactions();

  if (reactions.size() > 1) {
    auto it = reactions.begin();
    auto next = std::next(it);
    while (next != reactions.end()) {
      try_contract_edge(vertex_map.at(*it), vertex_map.at(*next));
      it = next;
      next = std::next(it);
    }
  }

  for (const auto* r : reactor->reactors()) {
    group_reactions_by_container_helper(r);
  }
}

auto GroupedDependencyGraph::transitive_reduction() -> GroupedDependencyGraph {
  GroupedDependencyGraph reduced{};

  // transitive_reduction uses this to populate a mapping from original vertices to new vertices in the reduced graph
  std::map<GroupGraph::vertex_descriptor, GroupGraph::vertex_descriptor> graph_to_reduced_graph{};
  // transitive reduction needs this mapping of vertices to integers
  std::map<GroupGraph::vertex_descriptor, std::size_t> id_map{};
  size_t id{0};
  for (GroupGraph::vertex_descriptor vd : boost::make_iterator_range(vertices(graph))) {
    id_map[vd] = id++;
  }

  // perform the actual reduction
  boost::transitive_reduction(graph, reduced.graph, make_assoc_property_map(graph_to_reduced_graph),
                              make_assoc_property_map(id_map));

  // update the mapping of reactions to vertices
  for (GroupGraph::vertex_descriptor vd : boost::make_iterator_range(vertices(graph))) {
    auto& reactions = boost::get(get_group_property_map(), vd);
    auto& reduced_reactions = boost::get(reduced.get_group_property_map(), graph_to_reduced_graph[vd]);
    reduced_reactions.insert(reduced_reactions.end(), reactions.begin(), reactions.end());
  }

  return reduced;
}

void GroupedDependencyGraph::group_chains() {
  for (GroupGraph::vertex_descriptor vd : boost::make_iterator_range(vertices(graph))) {
    if (in_degree(vd, graph) == 1) {
      GroupGraph::vertex_descriptor vs = source(*in_edges(vd, graph).first, graph);
      if (out_degree(vs, graph) == 1) {
        try_contract_edge(vs, vd);
      }
    }
  }

  clear_all_empty_vertices();
}

#ifndef __clang_analyzer__ // exclude this from static analysis as breadth_first_search appears to include a bug...
auto GroupedDependencyGraph::has_path(GroupGraph::vertex_descriptor va, GroupGraph::vertex_descriptor vb) const
    -> bool {
  try {
    breadth_first_search(graph, va, visitor(ReachabilityVisitor(vb)));
  } catch (const ReachabilityVisitor::PathExists& e) {
    return true;
  }
  return false;
}
#endif

} // namespace reactor
