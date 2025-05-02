#ifndef PDBS_CANONICAL_SSP_PDBS_H_PDB
#define PDBS_CANONICAL_SSP_PDBS_H_PDB

#include <memory>

#include "../representations/planning_utils.h"
#include "../representations/sasplus.h"
#include "types.h"

// using Problem = SasPlus::SasPlusMOSSP;
// using State = Problem::State;
using planning_utils::Cost;

namespace ssp_pdbs {
  class CanonicalPDBs {
    size_t num_objectives;
    std::shared_ptr<PDBCollection> pdbs;
    std::shared_ptr<std::vector<PatternClique>> pattern_cliques;

   public:
    CanonicalPDBs() = default;

    CanonicalPDBs(
            size_t num_objectives, const std::shared_ptr<PDBCollection> &pdbs,
            const std::shared_ptr<std::vector<PatternClique>> &pattern_cliques);

    ~CanonicalPDBs() = default;

    double get_value(const State &state) const;
  };
}

#endif
