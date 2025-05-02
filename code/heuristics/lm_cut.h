#ifndef HEURISTIC_LMCUT_H
#define HEURISTIC_LMCUT_H

#include <iostream>
#include <bitset>
#include <algorithm>  // std::min

#include "determinization_based_atom_abc.h"
#include "h_max.h"

#include "../ext/mgpt/states.h"
#include "../ext/mgpt/problems.h"

#include <boost/dynamic_bitset.hpp>

#ifndef LMCUT_MAX_ACTIONS
#define LMCUT_MAX_ACTIONS 8192
#endif

// using VectorOfBits = std::bitset<LMCUT_MAX_ACTIONS>;
using VectorOfBits = boost::dynamic_bitset<>;

#ifndef LMCUT_MAX_ATOMS
#define LMCUT_MAX_ATOMS 1024
#endif
// using VectorOfBitsAtm = std::bitset<LMCUT_MAX_ATOMS>;
using VectorOfBitsAtm = boost::dynamic_bitset<>;

/*******************************************************************************
 *
 * LM-Cut Heuristic
 *
 * Adapted from Patrik's HSP (http://users.cecs.anu.edu.au/~patrik/un-hsps.html)
 * specifically CostTable::compute_lmcut
 *
 ******************************************************************************/
class LMCutHeuristic : public determinizationBasedAtomHeuristic {
 public:
  LMCutHeuristic(problem_t const& problem, size_t cost_idx = ACTION_COST);
  ~LMCutHeuristic() { }

 protected:
  /*
   * heuristic_t interface
   */
  double computeValue(state_t const& s) override {
    if (relaxation_->goalT().holds(s, relaxation_->nprec())) {
      // This is a goal state in the relaxation, so returning 0
      return 0.0;
    }
    double v = computeLMCut(s, relaxation_->goalT().atom_list(0));
//    std::cout << "Hlmcut_{" << gpt::problem->domain().functions().name(cost_idx_)
//              << "}(" << s.toStringFull(gpt::problem) << ") = " << v
//              << std::endl;
    return v;
  }


  /*
   * determinizationBasedAtomHeuristic interface
   */
  double costSetOfAtoms(atomList_t const& atoms) const override {
    return std::min(maxCost(atoms, atom_rp_cost), dead_end_value_);
  }

 private:

  /*
   * Computes the LMCut heuristic from s to make all atoms in target_atoms true
   */
  double computeLMCut(state_t const& s, atomList_t const& target_atoms);

  void extendedGoals(atomList_t const& target_atoms,
                           std::vector<double> const& action_cost,
                           VectorOfBitsAtm& rv) const
  {
    extendedGoalsRec(target_atoms, action_cost, rv);
  }

  void extendedGoalsRec(atomList_t const& target_atoms,
                        std::vector<double> const& action_cost,
                        VectorOfBitsAtm& ext_goal_set) const;

  VectorOfBits findCut(VectorOfBitsAtm const& ext_goal_set,
                            std::vector<double> const& action_cost) const;

  // If allowed_actions is nullptr, then all actions are considered
  void computeCostOfAtomsForConstrainedActionSet(
                              state_t const& s,
                              std::vector<double> const& action_cost,
                              VectorOfBits const* allowed_actions = nullptr);

  /*
   * Members
   */
  // Vectors for caching the preconditions and add effects of actions as bitset.
  // This is useful because speed up intersection operations.
  //
  // TODO(fwt): use boost::dynamic_bitset
  std::vector<VectorOfBitsAtm> prec_op_vbits_;
  std::vector<VectorOfBitsAtm> adds_op_vbits_;
  MaxCostSetOfAtoms maxCost;
};

#endif  // HEURISTIC_LMCUT_H
