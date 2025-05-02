#include "causal_graph.h"

#include "../representations/sasplus.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>


using Action = ProblemDet::Action;
using Variable = SasPlus::SasPlusVariable;

using namespace std;

namespace causal_graph_ssp {

/*
  An IntRelationBuilder constructs an IntRelation by adding one pair
  to the relation at a time. Duplicates are automatically handled
  (i.e., it is OK to add the same pair twice), and the pairs need not
  be added in any specific sorted order.

  Define the following parameters:
  - K: range of the IntRelation (i.e., allowed values {0, ..., K - 1})
  - M: number of pairs added to the relation (including duplicates)
  - N: number of unique pairs in the final relation
  - D: maximal number of unique elements (x, y) in the relation for given x

  Then we get:
  - O(K + N) memory usage during construction and for final IntRelation
  - O(K + M + N log D) construction time
*/

class IntRelationBuilder {
  typedef unordered_set<int> IntSet;
  vector<IntSet> int_sets;

  int get_range() const;

  public:
  explicit IntRelationBuilder(int range);
  ~IntRelationBuilder();

  void add_pair(int u, int v);
  void compute_relation(IntRelation& result) const;
};

IntRelationBuilder::IntRelationBuilder(int range) : int_sets(range) {}

IntRelationBuilder::~IntRelationBuilder() {}

int IntRelationBuilder::get_range() const {
  return int_sets.size();
}

void IntRelationBuilder::add_pair(int u, int v) {
  assert(u >= 0 && u < get_range());
  assert(v >= 0 && v < get_range());
  int_sets[u].insert(v);
}

void IntRelationBuilder::compute_relation(IntRelation& result) const {
  int range = get_range();
  result.clear();
  result.resize(range);
  for (int i = 0; i < range; ++i) {
    result[i].assign(int_sets[i].begin(), int_sets[i].end());
    sort(result[i].begin(), result[i].end());
  }
}

struct CausalGraphBuilder {
  IntRelationBuilder pre_eff_builder;
  IntRelationBuilder eff_pre_builder;
  IntRelationBuilder eff_eff_builder;

  IntRelationBuilder succ_builder;
  IntRelationBuilder pred_builder;

  explicit CausalGraphBuilder(int var_count)
      : pre_eff_builder(var_count),
        eff_pre_builder(var_count),
        eff_eff_builder(var_count),
        succ_builder(var_count),
        pred_builder(var_count) {}

  ~CausalGraphBuilder() {}

  void handle_pre_eff_arc(int u, int v) {
    assert(u != v);
    pre_eff_builder.add_pair(u, v);
    succ_builder.add_pair(u, v);
    eff_pre_builder.add_pair(v, u);
    pred_builder.add_pair(v, u);
  }

  void handle_eff_eff_edge(int u, int v) {
    assert(u != v);
    eff_eff_builder.add_pair(u, v);
    eff_eff_builder.add_pair(v, u);
    succ_builder.add_pair(u, v);
    succ_builder.add_pair(v, u);
    pred_builder.add_pair(u, v);
    pred_builder.add_pair(v, u);
  }

  void handle_operator(const Action& op) {
    auto const& effect = op.effect();

    // Handle pre->eff links from preconditions.
    for (auto const& [pre_var_id, pre_val] : op.precondition()) {
      for (auto const& [eff_var_id, eff_val] : effect) {
        if (pre_var_id != eff_var_id)
          handle_pre_eff_arc(pre_var_id, eff_var_id);
      }
    }

    // Handle pre->eff links from effect conditions.
    // TODO the framework currently does not support effect conditions
    // for (EffectProxy eff : effects) {
    //   VariableProxy eff_var = eff.get_fact().get_variable();
    //   int eff_var_id = eff_var.get_id();
    //   for (FactProxy pre : eff.get_conditions()) {
    //     int pre_var_id = pre.get_variable().get_id();
    //     if (pre_var_id != eff_var_id)
    //       handle_pre_eff_arc(pre_var_id, eff_var_id);
    //   }
    // }

    // Handle eff->eff links.
    for (auto it1 = begin(effect); it1 != end(effect); ++it1) {
      int eff1_var_id = it1->first;
      for (auto it2 = begin(effect); it2 != end(effect); ++it2) {
        int eff2_var_id = it2->first;
        if (eff1_var_id != eff2_var_id)
          handle_eff_eff_edge(eff1_var_id, eff2_var_id);
      }
    }
  }
};

CausalGraph::CausalGraph(const ProblemDet& problem) {
  int num_variables = problem.numVariables();
  CausalGraphBuilder cg_builder(num_variables);

  for (Action const& op : problem.allActions()) {
    cg_builder.handle_operator(op);
  }

  // TODO the framework does currrently not support axioms
  // for (OperatorProxy op : task_proxy.get_axioms())
  //  cg_builder.handle_operator(op);

  cg_builder.pre_eff_builder.compute_relation(pre_to_eff);
  cg_builder.eff_pre_builder.compute_relation(eff_to_pre);
  cg_builder.eff_eff_builder.compute_relation(eff_to_eff);

  cg_builder.pred_builder.compute_relation(predecessors);
  cg_builder.succ_builder.compute_relation(successors);

  // dump(problem);
}

void CausalGraph::dump(const ProblemDet& problem) const {
  // fmt::print("Causal graph:\n");
  // for (Variable const& var : problem.variables()) {
  //   int var_id = var.index;
  //   fmt::print("#{} [{}]:\n", var_id, var.name);
  //   fmt::print("pre->eff arcs: {}\n", fmt::join(pre_to_eff[var_id], ","));
  //   fmt::print("eff->pre arcs: {}\n", fmt::join(eff_to_pre[var_id], ","));
  //   fmt::print("eff->eff arcs: {}\n", fmt::join(eff_to_eff[var_id], ","));
  //   fmt::print("successors: {}\n", fmt::join(successors[var_id], ","));
  //   fmt::print("predecessors: {}\n", fmt::join(predecessors[var_id], ","));
  // }
}

} // namespace causal_graph
