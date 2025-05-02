#ifndef PLANNER_FTVI_H
#define PLANNER_FTVI_H

#include <iostream>
#include <deque>
#include <stack>
#include <list>

#include "planner_iface.h"
#include "tvi.h"

#include "../ext/mgpt/actions.h"
#include "../ssps/ssp_adaptor.h"
#include "../ssps/ssp_iface.h"
#include "../ssps/ssp_utils.h"

#include "../heuristics/constant_value.h"  // for SmartConstantValueHeuristic

class problem_t;
class hash_t;
class heuristic_t;
class hashEntry_t;

#ifndef FTVI_INITIAL_NUM_PROB_DIST
#define FTVI_INITIAL_NUM_PROB_DIST 128
#endif

/*******************************************************************************
 *
 * planner FTVI
 *
 ******************************************************************************/
class PlannerFTVI : public OptimalPlanner
{
 public:
  PlannerFTVI(SSPIface const& ssp, heuristic_t& heur, double epsilon,
      // The default value of parameters are the values reported in the (F)TVI
      // paper on JAIR
      size_t num_search_ite_per_batch = 100,
      double lb_in_change_of_Vl_s0 = 0.03);

  ~PlannerFTVI() { }


  /*
   * Planner Interface
   */
  action_t const* decideAction(state_t const& s) override {
    // Assuming that s will be in the envelop computed by FTVI, which is more
    // than the envelop of the optimal policy but less than the whole reachable
    // space (since we might have pruned states reachable only by the pruned
    // suboptimal actions).
    solve();
    return Bellman::constGreedyAction(s, v_lb_, ssp_);
  }

  action_t const* decideAction(state_t const& s) const override {
    return Bellman::constGreedyAction(s, v_lb_, ssp_);
  }

  void trainForUsecs(uint64_t max_time_usec) override {
    if (!runForUsec(max_time_usec, [this] { solve(); })) {
      std::cout
        << "[FTVI::trainForUsecs]: training finished before convergence."
        << std::endl;
    }
  }

  void initRound() override { }
  void endRound() override { }
  void resetRoundStatistics() override { };
  void statistics(std::ostream& os, int level) const override { }

  /*
   * Heuristic Planner Interface
   */
  double value(state_t const& s) const override { return v_lb_.value(s); }

  /*
   * Optimal Planner Interface
   */
  double optimalSolution() override {
    solve();
    return v_lb_.value(ssp_.s0());
  }


 private:
  /*
   * FTVI methods
   */
  // Driver method that will solve the given SSP.
  void solve();

  /* 
   * Performs a best-first forward search from s. The base case (leaves) for
   * this search are goal states and states that have been visited before during
   * this search. Therefore, each state is updated at most once. It returns the
   * largest Bellman residual found during the search.
   *
   * As side-effect, the search procedure updates both the lower and upper
   * bound on V (v_lb_ and v_ub_ respectively) and adds any new suboptimal
   * action found to suboptimal_actions_
   *
   * FWT: See the commit 5150cc6deca1761ba9930e73c64aa6eac9d10d97 for the
   * original recursive version of this method. Notice that the recursive
   * version could easily use all the stack call and segfault. 
   */
  double search(state_t const& s, HashsetState& visited);

  double bellmanBackup(state_t const& s);

  // Method to count how many suboptimal actions were found. Used for statistics
  // only
  size_t totalSuboptimalActions() const {
    size_t t = 0;
    for (auto const& actions_from_s : suboptimal_actions_) {
      t += actions_from_s.second.size();
    }
    return t;
  }

  /*
   * Member variables
   */
  SSPIface const& ssp_;
  HashStateToActiontPtrs suboptimal_actions_;
  // Depends on suboptimal_actions_
  SSPAdaptor<SameGoals,
              ActionsToIgnoreFromHash,
              SameActionCost,
              SameTerminalCost,
              OnDemandReachableStates> pruned_problem_;

  // hash_t Ctor need a ref to heuristic_t, so we can't build it with an rvalue,
  // thus we need to keep a copy of the heuristic
  SmartConstantValueHeuristic h_ub_;
  hash_t v_lb_;
  hash_t v_ub_;
  double epsilon_;
  bool solved_;

  // x in the (F)TVI paper on JAIR
  size_t num_search_ite_per_batch_;
  // y/100 in the (F)TVI paper on JAIR
  double lb_in_change_of_Vl_s0_;

  // Here for efficiency reasons (instead of allocating/deallocating when
  // needed)
  ProbDistState pr_;
};

#endif  // PLANNER_FTVI_H
