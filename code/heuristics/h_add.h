#ifndef HEURISTICS_H_ADD
#define HEURISTICS_H_ADD

#include "determinization_based_atom_abc.h"

#include "../ext/mgpt/states.h"
#include "../ext/mgpt/problems.h"


/*
 * Aggregation functor for h-add
 */
struct SumCostSetOfAtoms {
  double operator()(atomList_t const& atoms, double* const& atom_rp_cost) const {
    double cost = 0;
    for (size_t i = 0; i < atoms.size(); ++i) {
      ushort_t a = atoms.atom(i);
      cost += atom_rp_cost[a];
    }
    return cost;
  }
};

/*
 * HAddAllOutcomesDet
 */
struct HAddAllOutcomesDetName {
  static std::string name() { return "h-add-all-out-det"; }
  static std::string nameShort() { return "h-add"; }
};
using HAddAllOutcomesDet = HDetAtomTemplate<SumCostSetOfAtoms,
                                            false,  // no self-loop relax
                                            HAddAllOutcomesDetName>;

/*
 * HAddSelfLoop
 */
struct HAddSelfLoopName {
  static std::string name() { return "h-add-selfloop"; }
  static std::string nameShort() { return "h-add-sl"; }
};
using HAddSelfLoop = HDetAtomTemplate<SumCostSetOfAtoms,
                                      true,  // using self-loop relax
                                      HAddSelfLoopName>;

#endif // HEURISTICS_H_ADD
