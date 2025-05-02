#include "canonical_ssp_pdbs_heuristic.h"

#include <iostream>
#include <limits>
#include <memory>

// #include "dominance_pruning.h"
#include "pattern_collection_generator_systematic.h"

using namespace std;

namespace ssp_pdbs {
CanonicalPDBs get_canonical_pdbs_from_options(Problem const& problem, int max_pattern_size) {
  PatternCollectionGeneratorSystematic pattern_generator(max_pattern_size);
  std::cout << "Initializing canonical PDB heuristic...\n";
  PatternCollectionInformation pattern_collection_info =
    pattern_generator.generate(problem);
  shared_ptr<PatternCollection> patterns =
    pattern_collection_info.get_patterns();
  /*
    We compute PDBs and pattern cliques here (if they have not been
    computed before) so that their computation is not taken into account
    for dominance pruning time.
  */
  shared_ptr<PDBCollection> pdbs = pattern_collection_info.get_pdbs();
  shared_ptr<std::vector<PatternClique>> pattern_cliques =
    pattern_collection_info.get_pattern_cliques();

  return CanonicalPDBs(problem.numCostFunctions(), pdbs, pattern_cliques);
}

CanonicalPDBsHeuristic::CanonicalPDBsHeuristic(Problem const& problem, int max_pattern_size) {
  canonical_pdbs = get_canonical_pdbs_from_options(problem, max_pattern_size);
}

double CanonicalPDBsHeuristic::heuristic(typename Problem::State const& state) {
  return canonical_pdbs.get_value(state);
}
} // namespace pdbs
