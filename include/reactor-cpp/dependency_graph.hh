#ifndef REACTOR_CPP_DEPENDENCY_GRAPH_HH
#define REACTOR_CPP_DEPENDENCY_GRAPH_HH

#include "reactor-cpp/reactor.hh"

#include <boost/graph/directed_graph.hpp>

namespace reactor {

class ReactionDependencyGraph {

private:
  struct reaction_info_t {
    using kind = boost::vertex_property_tag;
  };
  using ReactionProperty = boost::property<reaction_info_t, const Reaction*>;
  using ReactionGraph = boost::directed_graph<ReactionProperty>;
  using ReactionToVertexMap = std::map<const Reaction*, ReactionGraph::vertex_descriptor>;
  using ReactionPropertyMap = boost::property_map<ReactionGraph, reaction_info_t>::type;

      ReactionGraph graph{};
  ReactionToVertexMap vertex_map{};

  // helper functions
  void populate_graph_with_reactions(const Reactor* reactor);
  void populate_graph_with_priority_edges(const Reactor* reactor);
  void populate_graph_with_dependency_edges(const Reactor* reactor);

  [[nodiscard]] auto get_reaction_property_map() -> ReactionPropertyMap { return boost::get(reaction_info_t{}, graph); }

  ReactionDependencyGraph() = default;

public:
  ReactionDependencyGraph(const std::set<Reactor*>& top_level_reactors);

  void export_graphviz(const std::string& file_name);

  [[nodiscard]] auto transitive_reduction() const -> ReactionDependencyGraph;
};

} // namespace reactor

#endif
