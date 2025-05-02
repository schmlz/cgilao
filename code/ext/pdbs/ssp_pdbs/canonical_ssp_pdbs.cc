#include "canonical_ssp_pdbs.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>

#include "../representations/planning_utils.h"
#include "../../mgpt/global.h"
#include "pattern_database.h"

using namespace std;

using Problem = SasPlus::SasPlusMOSSP;
using State = Problem::State;

ssp_pdbs::CanonicalPDBs::CanonicalPDBs(
  size_t num_objectives, const shared_ptr<PDBCollection>& pdbs,
  const shared_ptr<vector<PatternClique>>& pattern_cliques)
    : num_objectives(num_objectives),
      pdbs(pdbs),
      pattern_cliques(pattern_cliques) {
  assert(pdbs);
  assert(pattern_cliques);
}

double ssp_pdbs::CanonicalPDBs::get_value(const State& state) const {
  // If we have an empty collection, then pattern_cliques = { \emptyset }.
  assert(!pattern_cliques->empty());
  vector<double> h_values = {};
  h_values.reserve(pdbs->size());
  for (const shared_ptr<PatternDatabase>& pdb : *pdbs) {  // PatternDatabase = database of values
    double const h = pdb->get_value(state);
    // If state is a dead end in any of the PDBs, then it must be a dead end for the SSP
    if (h >= gpt::dead_end_value.double_value()) {
      return gpt::dead_end_value.double_value();
    }
    h_values.push_back(h);
  }
  double max_h = 0.0;
  for (const PatternClique& clique : *pattern_cliques) {  // clique = additive set of patterns
    double clique_h = 0;
    for (PatternID pdb_index : clique) {
      clique_h += h_values[pdb_index];
    }
    max_h = std::max(max_h, clique_h);
  }
  return max_h;
}
