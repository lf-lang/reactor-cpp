#ifndef REACTOR_CPP_DEPENDENCY_GRAPH_HH
#define REACTOR_CPP_DEPENDENCY_GRAPH_HH

#include "reactor-cpp/reaction.hh"
#include "reactor-cpp/reactor.hh"

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/directed_graph.hpp>
#include <vector>

namespace reactor {

class GroupedDependencyGraph;

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

  friend GroupedDependencyGraph;
};

class GroupedDependencyGraph {

private:
  struct group_info_t {
    using kind = boost::vertex_property_tag;
  };
  using Group = std::vector<const Reaction*>;
  using GroupProperty = boost::property<group_info_t, Group>;
  using GroupGraph = boost::directed_graph<GroupProperty>;
  using ReactionToVertexMap = std::map<const Reaction*, GroupGraph::vertex_descriptor>;
  using GroupPropertyMap = boost::property_map<GroupGraph, group_info_t>::type;

  GroupGraph graph{};
  ReactionToVertexMap vertex_map{};

  [[nodiscard]] auto get_group_property_map() -> GroupPropertyMap { return boost::get(group_info_t{}, graph); }

  struct ReachabilityVisitor : public boost::default_bfs_visitor {
  private:
    GroupGraph::vertex_descriptor to;

  public:
    ReachabilityVisitor(GroupGraph::vertex_descriptor to)
        : to{to} {}

    // this exception is used to stop the BFS early if a matching vertex is found
    struct PathExists : public std::exception {};

    // this throws PathExists on success
    void discover_vertex(GroupGraph::vertex_descriptor u, [[maybe_unused]] const GroupGraph& g) const {
      if (u == to) {
        throw PathExists();
      }
    }
  };

  void group_reactions_by_container_helper(const Reactor* reactor);

  GroupedDependencyGraph() = default;

public:
  // TODO: This should be a const reference, but I don't know how to get immutable access to the reaction graph
  // properties...
  GroupedDependencyGraph(ReactionDependencyGraph& reactionGraph);

  void export_graphviz(const std::string& file_name);

  void try_contract_edge(const Reaction* a, const Reaction* b);

  void group_reactions_by_container(const std::set<Reactor*>& top_level_reactors);

  [[nodiscard]] auto transitive_reduction() const -> GroupedDependencyGraph;
};

} // namespace reactor

#endif
