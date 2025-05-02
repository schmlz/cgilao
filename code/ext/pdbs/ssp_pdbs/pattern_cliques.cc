#include "pattern_cliques.h"

#include "../representations/sasplus.h"
#include "max_cliques.h"
#include "pattern_database.h"
#include "../../mgpt/global.h"

using namespace std;

namespace ssp_pdbs {
bool are_patterns_additive(const Pattern& pattern1, const Pattern& pattern2,
                           const VariableAdditivity& are_additive) {
  for (int v1 : pattern1) {
    for (int v2 : pattern2) {
      if (!are_additive[v1][v2]) {
        return false;
      }
    }
  }
  return true;
}

VariableAdditivity compute_additive_vars(const ProblemDet& problem) {
  VariableAdditivity are_additive;
  int num_vars = problem.numVariables();
  are_additive.resize(num_vars, vector<bool>(num_vars, true));
  for (ProblemDet::Action const& op : problem.allActions()) {
    for (auto const& [e1_var_id, e1_val] : op.effect()) {
      for (auto const& [e2_var_id, e2_val] : op.effect()) {
        are_additive[e1_var_id][e2_var_id] = false;
      }
    }
  }
  return are_additive;
}

shared_ptr<vector<PatternClique>> compute_pattern_cliques(
  const PatternCollection& patterns, const VariableAdditivity& are_additive) {
  // Initialize compatibility graph.
  vector<vector<int>> cgraph;
  cgraph.resize(patterns.size());

  size_t deadline_counter = 0;
  for (size_t i = 0; i < patterns.size(); ++i) {
    for (size_t j = i + 1; j < patterns.size(); ++j) {
      if (are_patterns_additive(patterns[i], patterns[j], are_additive)) {
        /* If the two patterns are additive, there is an edge in the
           compatibility graph. */
        cgraph[i].push_back(j);
        cgraph[j].push_back(i);
      }
      gpt::incCounterAndCheckDeadlineEvery(deadline_counter, 10);
    }
  }

  shared_ptr<vector<PatternClique>> max_cliques =
    make_shared<vector<PatternClique>>();
  max_cliques_ssp::compute_max_cliques(cgraph, *max_cliques);
  return max_cliques;
}

vector<PatternClique> compute_pattern_cliques_with_pattern(
  const PatternCollection& patterns,
  const vector<PatternClique>& known_pattern_cliques,
  const Pattern& new_pattern, const VariableAdditivity& are_additive) {
  vector<PatternClique> cliques_additive_with_pattern;
  for (const PatternClique& known_clique : known_pattern_cliques) {
    // Take all patterns which are additive to new_pattern.
    PatternClique new_clique;
    new_clique.reserve(known_clique.size());
    for (PatternID pattern_id : known_clique) {
      if (are_patterns_additive(new_pattern, patterns[pattern_id],
                                are_additive)) {
        new_clique.push_back(pattern_id);
      }
    }
    if (!new_clique.empty()) {
      cliques_additive_with_pattern.push_back(new_clique);
    }
  }
  if (cliques_additive_with_pattern.empty()) {
    // If nothing was additive with the new variable, then
    // the only clique is the empty set.
    cliques_additive_with_pattern.emplace_back();
  }
  return cliques_additive_with_pattern;
}
} // namespace pdbs
