#ifndef PDBS_CANONICAL_SSP_PDBS_HEURISTIC_H_PDB
#define PDBS_CANONICAL_SSP_PDBS_HEURISTIC_H_PDB

#include "../representations/sasplus.h"
#include "canonical_ssp_pdbs.h"


// using Problem = SasPlus::SasPlusMOSSP;
// using State = Problem::State;
// using Cost = std::vector<double>;
// using Value = std::set<Cost>;

namespace ssp_pdbs {
// Implements the canonical heuristic function.
class CanonicalPDBsHeuristic {
  CanonicalPDBs canonical_pdbs;

 public:
  explicit CanonicalPDBsHeuristic(Problem const& problem, int max_pattern_size);
  virtual ~CanonicalPDBsHeuristic() = default;

  double heuristic(typename Problem::State const& s);
};
} // namespace pdbs

#endif
