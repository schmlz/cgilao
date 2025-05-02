#ifndef PDBS_TYPES_H_SSP_PDB
#define PDBS_TYPES_H_SSP_PDB

#include <memory>
#include <vector>
#include "../representations/sasplus.h"

namespace ssp_pdbs {

using Problem = SasPlus::SasPlusMOSSP;
using State = Problem::State;
using Cost = std::vector<double>;
using Value = std::set<Cost>;

class PatternDatabase;
using Pattern = std::vector<int>;
using PatternCollection = std::vector<Pattern>;
using PDBCollection = std::vector<std::shared_ptr<PatternDatabase>>;
using PatternID = int;
/* NOTE: pattern cliques are often called maximal additive pattern subsets
   in the literature. A pattern clique is an additive set of patterns,
   represented by their IDs (indices) in a pattern collection. */
using PatternClique = std::vector<PatternID>;
}

#endif
