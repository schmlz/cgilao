#ifndef PDBS_VALIDATION_H_SSP_PDB
#define PDBS_VALIDATION_H_SSP_PDB

#include "types.h"
#include "../representations/sasplus.h"


namespace ssp_pdbs {

using Problem = SasPlus::SasPlusMOSSP;
extern void validate_and_normalize_pattern(const Problem& problem,
                                           Pattern& pattern);
extern void validate_and_normalize_patterns(const Problem& problem,
                                            PatternCollection& patterns);
} // namespace pdbs

#endif
