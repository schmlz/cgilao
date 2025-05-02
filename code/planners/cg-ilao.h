#ifndef PLANNER_CGILAO
#define PLANNER_CGILAO

#include <iostream>
#include <unordered_map>
#include <algorithm>

#include "planner_iface.h"

#include "../ext/mgpt/actions.h"
#include "../ssps/bellman.h"
#include "../utils/die.h"
#include "../utils/mean_stdev_aggregator.h"
#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ssps/ssp_iface.h"
#include "../ssps/ssp_utils.h"

// #include "cg-dual-search-space.h"  // for the following:
// using SetOfStates = std::unordered_set<state_t, hashState>;
//
// struct StateActionPtr {
//   state_t state;
//   action_t const* action_ptr;
// };
//
// // Hash for StateAction. This is based on boost::hash_combine
// // http://www.boost.org/doc/libs/1_65_1/doc/html/hash/reference.html
// namespace std {
//   template<> struct hash<StateActionPtr> {
//     size_t operator()(StateActionPtr const& s_a) const {
//       size_t key = std::hash<state_t>()(s_a.state);
//       size_t action_ptr_key = std::hash<std::string>()(s_a.action_ptr ?
//                                                           s_a.action_ptr->name() : "");
//       key ^= action_ptr_key + 0x9e3779b9 + (key << 6) + (key >> 2);
//       return key;
//     }
//   };
//   template<> struct equal_to<StateActionPtr> {
//     bool operator()(StateActionPtr const& x, StateActionPtr const& y) const {
//       return (x.action_ptr == y.action_ptr) && (x.state == y.state);
//     }
//   };
// }
//
// using SetOfColumns = std::unordered_set<StateActionPtr>;

#include "cg-ilao-types.h"

class heuristic_t;

// #define D(code) { code; }
#ifndef D
#define D(code) { while(false) { } }
#endif

struct ImprovementStatus {
  bool stopped_due_policy_change;
  double max_residual;
};

enum class ExpansionType { bellman, bellman_tied, complete };

// TODO(fwt): maybe make this extend the SSPIface. Not really needed because I don't think we would
// be applying general algorithms to solve it
class PartialSSP {
 public:
  PartialSSP(SSPIface const& ssp);
  ~PartialSSP();

  bool containsState(state_t const& s) const {
    return partial_space_.find(s) != partial_space_.end();
  }

  bool isFringe(state_t const& s) const {
    auto const it = partial_space_.find(s);
    assert(it != partial_space_.end());
    return it->second.size() == 0;
  }

  bool isInternal(state_t const& s) const {
    assert(!ssp_.isGoal(s)); // FWT: Decide later what to do with goals
    auto const it = partial_space_.find(s);
    return it != partial_space_.end() && it->second.size() > 0;
  }

  bool isPartiallyExpanded(state_t const& s) const {
    auto const it = partial_space_.find(s);
    assert(it != partial_space_.end());
    size_t num_applicable = 0;
    for ([[maybe_unused]] action_t const& a : ssp_.applicableActions(s)) {
      num_applicable++;
    }
    if (num_applicable == 0) {
      // trivial dead end. The idea here is that a trivial dead end should
      // behave as a goal and always be fully expanded because nothing can
      // be done
      return false;
    }
    assert(it->second.size() <= num_applicable);
    return it->second.size() < num_applicable;
  }

  bool containsColumn(state_t const& s, action_t const& a) const {
    // make this check efficient
    auto const it = partial_space_.find(s);
    if (it == partial_space_.end()) { return false; }
    auto const jt = std::find(begin(it->second), end(it->second), &a);
    return jt != end(it->second);
  }

  void insertColumn(state_t const& s, action_t const* a);

  ImprovementStatus updateV(state_t const& s, action_t const* a_ptr, const double q_s_a, hash_t& v, SetOfConstrs& cols_to_be_checked);

  SetOfConstrs const& getInternalRegressionColumns(state_t const& s) const {
    return regression_partial_space_.at(s);
  }

  // Expand the state s which is supposed to be a fringe/artificial goal state using the Bellman
  // operator, i.e., it adds the column (s,a) where s is a fringe and a = argmin_{a} Q(s,a)
  // It *adds* to new_fringes the new reachable fringe states.
  void expandFringeWithBellman(state_t const& s, hash_t& v, ExpansionType const& expansion_type,
                               SetOfStates& new_fringes, SetOfStates* reached_non_fringe);

  // Simulates the current policy using DFS and populates the postorder_traversal. Only states where
  // an action **is applied** are in the postorder_traversal, i.e., goals, dead ends, and fringes
  // **are not** in the postorder_traversal.
  //
  // If expand_fringe_states is true, each fringe state found is expanded (just a single level, i.e.,
  // a fringe of a fringe is not expanded).
  void dfsSimilationOfCurPolicy(state_t const& s, hash_t& v, SetOfStates& new_fringes,
                                ListOfStates& postorder_traversal,
                                bool expand_fringe_states, ExpansionType const& expansion_type);

  // Expand partially expanded states s using the the action in the pair (s,a)
  void expandInternalStates(SetOfConstrs const& columns_to_be_added);

  // Performs VI over the partial SSP
  double valueIterationPartialSSP(hash_t& v, double epsilon);

  // Returns true if stopped early due to policy change. This only happens if stop_on_policy_changes
  // is also true
  ImprovementStatus greedyPolicyEvaluationPartialSSP(hash_t& v, double epsilon,
      bool stop_on_policy_changes, SetOfConstrs* cols_to_be_checked);

  void computeGreedyPolicyFringes(SetOfStates& new_fringes) {
    SetOfStates visited;
    new_fringes.clear();
    computeGreedyPolicyFringesRec(ssp_.s0(), cur_policy_, new_fringes, visited);
  }

  // Not const because it will do the lazy prunning of regression_missing_cols_
  void updateColumnsToBeChecked(SetOfStates const& v_increased, SetOfStates const& v_decreased,
      SetOfConstrs& cols_to_be_checked);

  // Computes the set of negative reduced cost columns with respect to v
  // Pinky-Promise: v is only changed when v(s) does not exist and it is populated with h(s)
  SetOfConstrs negativeReducedCostColumns(hash_t& v, SetOfConstrs const& cols_to_be_checked) const;

  // If cols_to_be_checked is not nullptr then it will be checked to completeness -- This should be
  // used for debugging or comparison only
  SetOfConstrs negativeReducedCostColumnsFullSearch(hash_t& v, SetOfConstrs const* cols_to_be_checked) const;

  action_t const* actionFor(state_t const& s) const {
    auto const it = cur_policy_.find(s);
    if (it == cur_policy_.end()) {
      // return nullptr;
      throw PlannerGaveUpException();
    }
    return it->second;
  }

  void printPartialSSPSize() const;
  void sparsityStatistics() const;
  void statistics() const;

  void dumpPartialSSPStateHistogram() const;
  void dumpPartialSSPActionHistogram() const;

  void dump(hash_t const* v = nullptr) const;
  void dumpPolicy(hash_t const* v) const {
    fancyPartialPolicyDebug(ssp_.s0(), v);
  }

  // Returns true if the full search for negative reduced cost columns find no columns. If any
  // column if found, it prints relevant information. This is a DEBUG method
  bool allNegativeReducedCostsAccountedFor(hash_t& v) const;

  // Performs a DFS on the current greedy policy and populate the postorder_traversal list. When a
  // fringe state is found, it expands with the greedy action only (i.e., partial expansion).
  // Notice that this method **ignores** the current fringe we kept track of.
  void dfsSimilationOfCurPolicyRec(state_t const& s, hash_t& v, SetOfStates& new_fringes, size_t depth,
      SetOfStates& open_or_closed, ListOfStates& postorder_traversal, bool expand_fringe_states,
      ExpansionType const& expansion_type);

  // Returns residual and if greedy policy changed. Also updates the cols_to_be_checked based on the
  // change of on V(s)
  ImprovementStatus partialBellmanBackup(state_t const& s, hash_t& v,
                                               SetOfConstrs& cols_to_be_checked);

  bool isCurPolicyClosed(hash_t const* v) const;

 private:
  // This method here to centralized policy changes to help debugging or any other operation needed
  // when the policy changes. There should be no penalty in performance
  void updateCurPolicy(state_t const& s, action_t const* a) {
    cur_policy_[s] = a;
  }

  void fancyPartialPolicyDebug(state_t const& s0, hash_t const* v) const;

  void fancyPolicyDebugRec(state_t const& s, HashsetState& open_or_closed, bool partial_pi,
                           hash_t const* v, std::string indentation) const;

  void computeGreedyPolicyFringesRec(state_t s, Policy const& pi,
                                     SetOfStates& new_fringes, SetOfStates& visited);

  SSPIface const& ssp_;

  // The keys are the union of the set of partially expanded, fully expanded and fringe states.
  // The values is the set of actions available for that state hat{A}:
  //  - fully expanded states: hat{A}(s) == A(s)
  //  - partially expanded states: hat{A}(s) \subset A(s)
  //  - fringe states: hat{A}(s) == empty_set
  //
  // TODO(fwt): what happens to goal states?
  MapStateToActionPtrs partial_space_;

  // This policy will store the FF action for a state s *if* pi(s) is NOT defined and it will be
  // updated with the best greedy action by valueIterationPartialSSP and
  // greedyPolicyEvaluationPartialSSP
  Policy cur_policy_;
  // Used to efficiently schedule only the missing columns that could potentially become negative to
  // have their reduced cost checked
  StateToConstrs regression_missing_cols_;
  // Used to propagate towards the initial state the decreased V(s) when a negative reduced cost
  // column is added
  StateToConstrs regression_partial_space_;

  VectorProbDistState v_pr_;
};


/*******************************************************************************
 *
 * planner CG-iLAO*: AAAI'24
 *
 ******************************************************************************/
class PlannerCGiLAO : public OptimalPlanner
{
 public:
  PlannerCGiLAO(SSPIface const& ssp, heuristic_t& heur, double epsilone);

  ~PlannerCGiLAO() {
    std::cout << "[total q-values] " << gpt::total_computed_qvalues << std::endl;
    for (auto const& pair : qvalues_computed_) {
      std::cout << "[q-values " << pair.first << "] " << pair.second << " ("
                << (100 * pair.second / (float) gpt::total_computed_qvalues) << "%)\n";
    }
  }

  /*
   * Planner Interface
   */
  action_t const* decideAction(state_t const& s) override {
    if (!solved_from_s0_) {
      solve();
    }
    return partial_ssp_.actionFor(s); // This is a const method, so it will not "replan"
  }

  action_t const* decideAction(state_t const& s) const override {
    /*
     * Bellman::constGreedyAction is not used here because we already have a
     * policy for the problem inside the explicit graph.
     */
    if (ssp_.isGoal(s) || !ssp_.hasApplicableActions(s)) return nullptr;

    // if (!explicit_graph_.hasEntryFor(s)) {
    //   // State was not explored by the search. This should not happen in a
    //   // regular situation since the explicit graph contains a closed policy
    //   // with respect to s0
    //   throw PlannerGaveUpException();
    // }
    // action_t const* a = explicit_graph_.actionForState(s);
    // if (a == nullptr) {
    //   DIE(!ssp_.isGoal(s) && ssp_.hasApplicableActions(s),
    //       "Unexpected goal/state without actions", -1);
    //   // Most likely the planner was interrupted before finding a closed
    //   // policy. If NULL is returned, then the round evaluation will be tagged
    //   // as dead end reached, what is not the case.
    //   throw PlannerGaveUpException();
    // }
    // _D(DEBUG_ILAO, std::cout << "  Best a = " << a->name() << std::endl);
    // return a;
    return partial_ssp_.actionFor(s);
  }

  void trainForUsecs(uint64_t max_time_usec) override {
    NOT_IMPLEMENTED;
    // if (!runForUsec(max_time_usec, [this]() { solve(); })) {
    //   std::cout << "[cg-ilao::trainForUsecs]: training finished before "
    //             << " convergence." << std::endl;
    // }
  }

  void initRound() override { }
  void endRound() override { }
  void resetRoundStatistics() override { };
  void statistics(std::ostream& os, int level) const override { }

  /*
   * Heuristic Planner Interface
   */
  double value(state_t const& s) const override { return v_.value(s); }

  /*
   * Optimal Planner Interface
   */
  double optimalSolution() override {
    if (!solved_from_s0_) {
      solve();
    }
    return v_.value(ssp_.s0());
  }

 private:

  // Main method.
  void solve();
  // Returns the number of open states found in cur_policy_ and populate the postorder_traversal
  // (fringes and goals ommitted)
  size_t expandPolicy(ListOfStates& postorder_traversal);
  // Perform Bellman backups (either in the postorder_traversal, full VI or an hybrid) and return
  // the reason the optimization stopped. is_policy_open is needed because the open states are not
  // included in the postorder_traversal
  ImprovementStatus improvePolicy(ListOfStates& postorder_traversal,
                                  bool is_policy_open,
                                  SetOfConstrs& cols_to_be_checked);

  ImprovementStatus fixViolatedConstraints(SetOfConstrs& cols_to_be_checked);


  /*
   * Member variables
   */
  SSPIface const& ssp_;
  hash_t v_;

  double epsilon_;

  // Internal DS
  PartialSSP partial_ssp_;

  bool solved_from_s0_;

  ExpansionType expansion_;

  std::unordered_map<std::string, uint64_t> cputime_;
  std::unordered_map<std::string, size_t> qvalues_computed_;
};

#endif  // PLANNER_CGILAO
