#ifndef PLANNER_CGILAO_EXTENDED
#define PLANNER_CGILAO_EXTENDED

#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <optional>

#include "planner_iface.h"

#include "../ext/mgpt/actions.h"
#include "../ssps/bellman.h"
#include "../utils/die.h"
#include "../utils/mean_stdev_aggregator.h"
#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ssps/ssp_iface.h"
#include "../ssps/ssp_utils.h"
#include "../ext/det_planners/externalDetPlannerInterface.h"

#include "../heuristics/constant_value.h" // for v_ub_

#include "cg-ilao-types.h"

class heuristic_t;

// #define D(code) { code; }
#ifndef D
#define D(code) { while(false) { } }
#endif

struct CGiLAOExtendedImprovementStatus {
  bool stopped_due_policy_change;
  double max_residual;
};

enum class CGiLAOExtendedExpansionType { bellman, ff, bellman_tied, bellman_n_best, bellman_x_best, bellman_x2_best, bellman_x3_best, complete, trial };

// TODO(fwt): maybe make this extend the SSPIface. Not really needed because I don't think we would
// be applying general algorithms to solve it
class PartialSSPExtended {
 public:
  PartialSSPExtended(SSPIface const& ssp);
  ~PartialSSPExtended();

  void setExpansionSettings(CGiLAOExtendedExpansionType expansion, size_t n, double x) {
    expansion_ = expansion;
    expansion_n_ = n;
    expansion_x_ = x;
    expansion_set_up_ = true;
  }

  size_t getMaxUniqueQvalues() const {
    return max_unique_qvalues_;
  }

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

  size_t numApplicableActions(state_t const& s) {
    // size_t num_applicable = 0;
    // for ([[maybe_unused]] action_t const& a : ssp_.applicableActions(s)) {
    //   num_applicable++;
    // }
    // return num_applicable;

    if (num_actions_cache_.find(s) == num_actions_cache_.end()) {
      size_t num_applicable = 0;
      for ([[maybe_unused]] action_t const& a : ssp_.applicableActions(s)) {
        num_applicable++;
      }
      num_actions_cache_[s] = num_applicable;
    }
    return num_actions_cache_[s];

  }

  bool isPartiallyExpanded(state_t const& s) {
    auto const it = partial_space_.find(s);
    assert(it != partial_space_.end());

    size_t num_applicable = numApplicableActions(s);

    if (num_applicable == 0) {
      // trivial dead end. The idea here is that a trivial dead end should
      // behave as a goal and always be fully expanded because nothing can
      // be done
      return false;
    }
    assert(it->second.size() <= num_applicable);
    return it->second.size() < num_applicable;
  }

  VecActionPtr& externalActions(state_t const& s) {
    if (external_space_.find(s) == external_space_.end()) {
      for (action_t const& a : ssp_.applicableActions(s)) {
        if (!containsColumn(s, a)) {
          external_space_[s].emplace_back(&a);
        }
      }
    }
    return external_space_[s];
  }

  bool containsColumn(state_t const& s, action_t const& a) const {
    // make this check efficient
    auto const it = partial_space_.find(s);
    if (it == partial_space_.end()) { return false; }
    auto const jt = std::find(begin(it->second), end(it->second), &a);
    return jt != end(it->second);
  }

  void insertColumn(state_t const& s, action_t const* a);

  void removeColumn(state_t const& s, action_t const* a);

  size_t removeStaleColumns(int const max_age);

  CGiLAOExtendedImprovementStatus updateV(state_t const& s, action_t const* a_ptr, const double q_s_a, hash_t& v, SetOfConstrs& cols_to_be_checked);

  SetOfConstrs const& getInternalRegressionColumns(state_t const& s) const {
    return regression_partial_space_.at(s);
  }

  // Get "best goals" (based on the definition from Robust-FF)
  std::vector<state_t> getBestGoals(hash_t const& v, const size_t n_best_goals) const;


  DetPlannerReturnType runTrialFrom(state_t const& s,
                                    std::vector<std::pair<double, state_t>>& state_trace,
                                    std::vector<action_t const*>& action_trace, hash_t& v,
                                    size_t const max_trial_steps);

  DetPlannerReturnType runDetPlannerFrom(state_t const& s,
                                         std::vector<std::pair<double, state_t>>& state_trace,
                                         std::vector<action_t const*> & action_trace,
                                         const bool use_mlo,
                                         std::vector<state_t> const& extra_goals,
                                         const std::string det_planner = "ff");

  void markDeadEnd(state_t const& s, hash_t& v);
  void markReachableFringesAndNonFringes(state_t const& starting_s,
                                        std::vector<std::pair<double, state_t>> const& state_trace,
                                        std::vector<action_t const*> const& action_trace,
                                        SetOfStates& reachable_fringes,
                                        SetOfStates& reachable_non_fringes);

  // Expand the state s which is supposed to be a fringe/artificial goal state using FF over the
  // all-outcomes determinization. It *adds* to new_fringes the new reachable fringe states
  void expandFringesWithFF(state_t const& s, hash_t& v, SetOfStates& new_fringes, const bool use_mlo,
                           std::vector<state_t> const& extra_goals, SetOfStates& reached_non_fringe);


  void expandFringesWithTrial(state_t const& s, hash_t& v, SetOfStates& new_fringes, SetOfStates& reached_non_fringe, size_t const max_trial_steps);

  // Expand the state s which is supposed to be a fringe/artificial goal state using the Bellman
  // operator, i.e., it adds the column (s,a) where s is a fringe and a = argmin_{a} Q(s,a)
  // It *adds* to new_fringes the new reachable fringe states.
  void expandFringeWithBellman(state_t const& s, hash_t& v, CGiLAOExtendedExpansionType const& expansion_type,
                               SetOfStates& new_fringes, SetOfStates* reached_non_fringe);

  // Simulates the current policy using DFS and populates the postorder_traversal. Only states where
  // an action **is applied** are in the postorder_traversal, i.e., goals, dead ends, and fringes
  // **are not** in the postorder_traversal.
  //
  // If expand_fringe_states is true, each fringe state found is expanded (just a single level, i.e.,
  // a fringe of a fringe is not expanded).
  void dfsSimilationOfCurPolicy(state_t const& s, hash_t& v, SetOfStates& new_fringes,
                                ListOfStates& postorder_traversal,
                                bool expand_fringe_states, CGiLAOExtendedExpansionType const& expansion_type);

  // Expand partially expanded states s using the the action in the pair (s,a)
  void expandInternalStates(SetOfConstrs const& columns_to_be_added);

  // Performs VI over the partial SSP
  CGiLAOExtendedImprovementStatus valueIterationPartialSSPExtended(hash_t& v, double epsilon, SetOfConstrs& cols_to_be_checked);

  action_t const* actionFor(state_t const& s) const {
    auto const it = cur_policy_.find(s);
    if (it == cur_policy_.end()) {
      // return nullptr;
      throw PlannerGaveUpException();
    }
    return it->second;
  }

  void statistics() const;

  void dump(hash_t const* v = nullptr) const;
  void dumpPolicy(hash_t const* v) const {
    fancyPartialPolicyDebug(ssp_.s0(), v);
  }

  void printPartialSSPSize() const;
  void sparsityStatistics() const;

  // Performs a DFS on the current greedy policy and populate the postorder_traversal list. When a
  // fringe state is found, it expands with the greedy action only (i.e., partial expansion).
  // Notice that this method **ignores** the current fringe we kept track of.
  void dfsSimilationOfCurPolicyRec(state_t const& s, hash_t& v, SetOfStates& new_fringes, size_t depth,
      SetOfStates& open_or_closed, ListOfStates& postorder_traversal, bool expand_fringe_states,
      CGiLAOExtendedExpansionType const& expansion_type);

  // Returns residual and if greedy policy changed. Also updates the cols_to_be_checked based on the
  // change of on V(s)
  CGiLAOExtendedImprovementStatus partialBellmanBackup(state_t const& s, hash_t& v,
                                               SetOfConstrs& cols_to_be_checked,
                                               bool const track_age);

  size_t getNColsAdded() const { return n_columns_added_; }
  size_t getNColsInPartSSP() const {
    size_t n = 0;
    for (auto const& [s, acts] : partial_space_) {
      n += acts.size();
    }
    return n;
  }

  // for action elimination
  void eliminateColumn(state_t const& s, action_t const* a, SetOfConstrs& cols_to_be_checked) {
    assert(eliminated_actions_.find(s) == eliminated_actions_.end() or
           eliminated_actions_.at(s).find(a) == eliminated_actions_.at(s).end());
    removeColumn(s, a);
    eliminated_actions_[s].emplace(a);
    n_eliminated_actions_ += 1;

    // TODO: would be nice to do this book-keeping here, but this is buggy
    // --> for now just checking isEliminated where relevant
    //
    // cols_to_be_checked.erase({s, a});
    // regression_missing_cols_[s].erase({s, a});
  }
  bool isEliminated(state_t const& s, action_t const* a) {
    auto it = eliminated_actions_.find(s);
    if (it == eliminated_actions_.end()) {
      return false;
    }
    auto it_inner = it->second.find(a);
    return (it_inner == it->second.end());
  }
  bool isEliminated(StateActionPtr const& col) {
    return isEliminated(col.state, col.action_ptr);
  }
  size_t getNEliminatedActions() const {
    return n_eliminated_actions_;
  }
  void setActionEliminationEnabled(bool const x) { action_elim_enabled_ = x; }
  void resetActionEliminationResidual() {
    // IMPORTANT: this residual is not (currently) used for planning, it's just to see how accuarate
    // the UB is
    v_ub_residual_ = 0.0;
  }
  void printActionEliminationInfo() {
    if (!action_elim_enabled_) {
      return;
    }

    if (!v_ub_.has_value()) {
      std::cout << "V_ub(s0) = undefined" << std::endl;
    } else {
      std::cout << "V_ub(s0) = " << v_ub_.value().value(ssp_.s0()) << std::endl;
      std::cout << "V_ub residual = " << v_ub_residual_ << std::endl;
    }
  }

 private:
  // This method here to centralized policy changes to help debugging or any other operation needed
  // when the policy changes. There should be no penalty in performance
  void updateCurPolicy(state_t const& s, action_t const* a) {
    cur_policy_[s] = a;
  }

  void fancyPartialPolicyDebug(state_t const& s0, hash_t const* v) const;

  void fancyPolicyDebugRec(state_t const& s, HashsetState& open_or_closed, bool partial_pi,
                           hash_t const* v, std::string indentation) const;

  SSPIface const& ssp_;

  // Settings for Bellman expansions
  CGiLAOExtendedExpansionType expansion_;
  size_t expansion_n_;
  double expansion_x_;
  bool expansion_set_up_ = false;  // HACK(jsch): flag for checking if things were set up properly
  size_t max_unique_qvalues_ = 0;

  // The keys are the union of the set of partially expanded, fully expanded and fringe states.
  // The values is the set of actions available for that state hat{A}:
  //  - fully expanded states: hat{A}(s) == A(s)
  //  - partially expanded states: hat{A}(s) \subset A(s)
  //  - fringe states: hat{A}(s) == empty_set
  //
  // TODO(fwt): what happens to goal states?
  MapStateToActionPtrs partial_space_;

  MapStateToActionPtrs external_space_;

  std::shared_ptr<ExternalDetPlannerInterface> external_det_planner_ao_;
  std::shared_ptr<ExternalDetPlannerInterface> external_det_planner_mlo_;

  // This policy will store the FF action for a state s *if* pi(s) is NOT defined and it will be
  // updated with the best greedy action by valueIterationPartialSSPExtended and
  // greedyPolicyEvaluationPartialSSPExtended
  Policy cur_policy_;
  // Used to efficiently schedule only the missing columns that could potentially become negative to
  // have their reduced cost checked
  StateToConstrs regression_missing_cols_;
  // Used to propagate towards the initial state the decreased V(s) when a negative reduced cost
  // column is added
  StateToConstrs regression_partial_space_;

  // this is NOT the number of columns in part. SSP, the same col may be added multiple times with
  // action removal!
  size_t n_columns_added_ = 0;

  std::unordered_map<StateActionPtr, int> column_ages_;

  VectorProbDistState v_pr_;

  uint64_t ff_cputime_usecs_ = 0;

  // For action elimination
  std::optional<SmartConstantValueHeuristic> h_ub_;
  std::optional<hash_t> v_ub_;
  double v_ub_residual_ = 0.0;
  std::unordered_map<state_t, std::unordered_set<action_t const*>> eliminated_actions_;
  size_t n_eliminated_actions_ = 0;
  bool action_elim_enabled_ = false;
  bool has_seen_goal_ = false;

  std::unordered_map<state_t, size_t> num_actions_cache_;
};


/*******************************************************************************
 *
 * planner CG-iLAO* with extensions: JAIR'24
 *
 ******************************************************************************/
class PlannerCGiLAOExtended : public OptimalPlanner
{
 public:
  PlannerCGiLAOExtended(SSPIface const& ssp, heuristic_t& heur, double epsilon,
      std::string const& k, std::string const& expansion_type, std::string const& improvement_type,
      std::string const& n_violation_fix_passes, std::string const& max_col_age,
      std::string const& action_elim);

  ~PlannerCGiLAOExtended() {
    std::cout << "[total q-values] " << gpt::total_computed_qvalues << std::endl;
    for (auto const& pair : qvalues_computed_) {
      std::cout << "[q-values " << pair.first << "] " << pair.second << " ("
                << (100 * pair.second / (float) gpt::total_computed_qvalues) << "%)\n";
    }
    std::cout << "[expandPolicy expansions_performed] " << stats_about_k_ << std::endl;
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

  bool isExpansionFromBellmanFamily() const {
    return expansion_ == CGiLAOExtendedExpansionType::bellman \
            || expansion_ == CGiLAOExtendedExpansionType::bellman_tied \
            || expansion_ == CGiLAOExtendedExpansionType::complete \
            || expansion_ == CGiLAOExtendedExpansionType::bellman_n_best \
            || expansion_ == CGiLAOExtendedExpansionType::bellman_x_best \
            || expansion_ == CGiLAOExtendedExpansionType::bellman_x2_best \
            || expansion_ == CGiLAOExtendedExpansionType::bellman_x3_best;
  }


 private:
  enum class ImprovementType { postorder, vi };

  // Main method.
  void solve();
  using ExpandFringeFunction = std::function<void(state_t const& s, hash_t& v, SetOfStates& new_fringes, SetOfStates& reached_non_fringe)>;
  using ExpandPolicyFunction = std::function<void(ListOfStates& postorder_traversal)>;
  bool debugIsSetSupersetOfFringe(SetOfStates const& set);
  // Returns the number of open states found in cur_policy_ and populate the postorder_traversal
  // (fringes and goals ommitted)
  void expandPolicyInnerLoop(size_t n_expansions, ExpandFringeFunction& expand_func);
  void expandPolicyBellmanK1(ListOfStates& postorder_traversal);
  void expandPolicyBellmanKMoreThan1(ListOfStates& postorder_traversal);
  void expandPolicyFF(ListOfStates& postorder_traversal);
  void expandPolicyTrial(ListOfStates& postorder_traversal);
  size_t expandPolicy(ListOfStates& postorder_traversal);
  // Perform Bellman backups (either in the postorder_traversal, full VI or an hybrid) and return
  // the reason the optimization stopped. is_policy_open is needed because the open states are not
  // included in the postorder_traversal
  CGiLAOExtendedImprovementStatus improvePolicy(ListOfStates& postorder_traversal,
                                  bool is_policy_open,
                                  SetOfConstrs& cols_to_be_checked);

  CGiLAOExtendedImprovementStatus fixViolatedConstraints(SetOfConstrs& cols_to_be_checked);


  /*
   * Member variables
   */
  SSPIface const& ssp_;
  hash_t v_;

  double epsilon_;

  // Internal DS
  PartialSSPExtended partial_ssp_;

  bool solved_from_s0_;

  size_t k_;
  bool use_infty_k_;
  CGiLAOExtendedExpansionType expansion_;
  size_t expansion_n_;
  double expansion_x_;
  ImprovementType improvement_;
  size_t n_violation_fix_passes_;  // j
  bool use_infty_n_violation_fix_passes_;
  size_t fix_constrs_gap_;  // j'
  bool use_infty_fix_constrs_gap_;
  int max_col_age_;

  // FF settings
  bool use_mlo_;
  size_t n_best_goals_;

  // Trial settings
  size_t max_trial_steps_ = 10;

  // For policy expansion
  // FWT: alternating between the two to avoid copy constructor
  SetOfStates policy_fringe_[2];
  bool fringe_idx_ = 0;
  ExpandPolicyFunction expand_policy_func_;


  std::unordered_map<std::string, uint64_t> cputime_;
  std::unordered_map<std::string, size_t> qvalues_computed_;
  MeanStdevAggregator stats_about_k_;
  double init_v_s0_;
  size_t n_stale_col_removals_ = 0;
};

#endif  // PLANNER_CGILAO_EXTENDED
