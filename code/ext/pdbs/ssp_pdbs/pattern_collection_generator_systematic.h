#ifndef PDBS_PATTERN_COLLECTION_GENERATOR_SYSTEMATIC_H_SSP_PDB
#define PDBS_PATTERN_COLLECTION_GENERATOR_SYSTEMATIC_H_SSP_PDB

#include <cstdlib>
#include <memory>
#include <unordered_set>
#include <vector>

#include "../representations/sasplus.h"
#include "hash.h"
#include "pattern_collection_information.h"
#include "types.h"

namespace causal_graph_ssp {
class CausalGraph;
}

namespace ssp_pdbs {
class CanonicalPDBsHeuristic;

using Problem = SasPlus::SasPlusMOSSP;

// Invariant: patterns are always sorted.
class PatternCollectionGeneratorSystematic {
  using PatternSet = utils::HashSet<Pattern>;

  const size_t max_pattern_size;
  std::shared_ptr<PatternCollection> patterns;
  PatternSet pattern_set; // Cleared after pattern computation.

  void enqueue_pattern_if_new(const Pattern& pattern);
  void compute_eff_pre_neighbors(const causal_graph_ssp::CausalGraph& cg,
                                 const Pattern& pattern,
                                 std::vector<int>& result) const;
  void compute_connection_points(const causal_graph_ssp::CausalGraph& cg,
                                 const Pattern& pattern,
                                 std::vector<int>& result) const;

  void build_sga_patterns(const Problem& problem,
                          const causal_graph_ssp::CausalGraph& cg);
  void build_patterns(Problem problem);

  public:
  explicit PatternCollectionGeneratorSystematic(int max_pattern_size);

  PatternCollectionInformation generate(Problem const& problem);
};
} // namespace pdbs

#endif
