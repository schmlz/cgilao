#include "pattern_collection_information.h"

#include <algorithm>
#include <cassert>
#include <unordered_set>
#include <utility>

#include "pattern_cliques.h"
#include "pattern_database.h"
#include "validation.h"

#include "../../mgpt/global.h"

using namespace std;

namespace ssp_pdbs {
PatternCollectionInformation::PatternCollectionInformation(
  const Problem& problem, const shared_ptr<PatternCollection>& patterns)
    : problem(problem),
      patterns(patterns),
      pdbs(nullptr),
      pattern_cliques(nullptr) {
  assert(patterns);
  validate_and_normalize_patterns(problem, *patterns);
}

bool PatternCollectionInformation::information_is_valid() const {
  if (!patterns) {
    return false;
  }
  int num_patterns = patterns->size();
  if (pdbs) {
    if (patterns->size() != pdbs->size()) {
      return false;
    }
    for (int i = 0; i < num_patterns; ++i) {
      if ((*patterns)[i] != (*pdbs)[i]->get_pattern()) {
        return false;
      }
    }
  }
  if (pattern_cliques) {
    for (const PatternClique& clique : *pattern_cliques) {
      for (PatternID pattern_id : clique) {
        if (!in_bounds(pattern_id, *patterns)) {
          return false;
        }
      }
    }
  }
  return true;
}

void PatternCollectionInformation::create_pdbs_if_missing() {
  assert(patterns);
  if (!pdbs) {
    std::cout << "Computing PDBs for pattern collection...\n";
    pdbs = make_shared<PDBCollection>();
    for (const Pattern& pattern : *patterns) {
      shared_ptr<PatternDatabase> pdb = make_shared<PatternDatabase>(problem, pattern);
      pdbs->push_back(pdb);
      gpt::checkDeadline();
    }
    std::cout << "Done computing PDBs for pattern collection...\n";
  }
}

void PatternCollectionInformation::create_pattern_cliques_if_missing() {
  if (!pattern_cliques) {
    std::cout << "Computing pattern cliques for pattern collection...\n";
    VariableAdditivity are_additive = compute_additive_vars(problem.determinise());
    pattern_cliques = compute_pattern_cliques(*patterns, are_additive);
    std::cout << "Done computing pattern cliques for pattern collection.\n";
  }
}

void PatternCollectionInformation::set_pdbs(
  const shared_ptr<PDBCollection>& pdbs_) {
  pdbs = pdbs_;
  assert(information_is_valid());
}

void PatternCollectionInformation::set_pattern_cliques(
  const shared_ptr<vector<PatternClique>>& pattern_cliques_) {
  pattern_cliques = pattern_cliques_;
  assert(information_is_valid());
}

shared_ptr<PatternCollection> PatternCollectionInformation::get_patterns()
  const {
  assert(patterns);
  return patterns;
}

shared_ptr<PDBCollection> PatternCollectionInformation::get_pdbs() {
  create_pdbs_if_missing();
  return pdbs;
}

shared_ptr<vector<PatternClique>>
PatternCollectionInformation::get_pattern_cliques() {
  create_pattern_cliques_if_missing();
  return pattern_cliques;
}
} // namespace pdbs
