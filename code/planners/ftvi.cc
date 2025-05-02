#include <cmath>

#include "ftvi.h"
#include "tvi.h"

#include "../utils/die.h"
#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ssps/ssp_adaptor.h"
#include "../ext/mgpt/states.h"

#include "../heuristics/heuristic_iface.h"


/*******************************************************************************
 *
 * planner FTVI
 *
 ******************************************************************************/

PlannerFTVI::PlannerFTVI(SSPIface const& ssp, heuristic_t &heur,
    double epsilon, size_t num_search_ite_per_batch,
    double lb_in_change_of_Vl_s0)
  : OptimalPlanner(), ssp_(ssp),
    suboptimal_actions_(),
    pruned_problem_(ssp, "Pruned Suboptimal Actions SSP",
                        ssp.s0(), SameGoals(),
                        ActionsToIgnoreFromHash(suboptimal_actions_),
                        SameActionCost(), SameTerminalCost(),
                        OnDemandReachableStates()),
    h_ub_(ssp, gpt::dead_end_value.double_value()),
    v_lb_(gpt::initial_hash_size, heur),
    v_ub_(gpt::initial_hash_size, h_ub_),
    epsilon_(epsilon), solved_(false),
    num_search_ite_per_batch_(num_search_ite_per_batch),
    lb_in_change_of_Vl_s0_(lb_in_change_of_Vl_s0)
{ }



void PlannerFTVI::solve() {
  if (solved_) return;

  HashsetState visited;
  state_t const& s0 = ssp_.s0();

  /*
   * Applying simple forward search. This helps by improving the lower bound on
   * V* and finding suboptimal actions
   */
  size_t batch_id = 0;
  while (true) {
    std::cout << "Starting Search Batch " << batch_id << std::endl;

    double old_v_s0 = v_lb_.value(s0);

    for (size_t i = 0; i < num_search_ite_per_batch_; ++i) {
      visited.clear();
      gpt::checkDeadline();

      if (search(s0, visited) <= epsilon_) {
        std::cout << "Search Batch " << batch_id
                  << " Found the Optimal Solution" << std::endl
                  << "[FTVI]: " << totalSuboptimalActions()
                  << " pairs in ignore action" << std::endl;
        // The forward search already found the solution
        solved_ = true;

        std::cout << "\nCSVHACK"
                  << "," << gpt::__ppddl_domain_filename
                  << "," << gpt::__ppddl_problem_filename
                  << "," << gpt::seed
                  << "," << gpt::algorithm
                  << "," << gpt::heuristic
                  << "," << v_lb_.value(ssp_.s0())
                  << "," << get_cputime_usec()
                  << "," << gpt::heur_ptr->totalCalls()
                  << "," << gpt::total_computed_qvalues
                  << "\n\n";

        return;
      }
    }

    std::cout << "Finishing Search Batch " << batch_id << ": "
              << "old_v_s0 = " << old_v_s0 << " cur_v_s0 = "
              << v_lb_.value(s0) << "\t ratio = "
              << (old_v_s0 / v_lb_.value(s0))
              << std::endl;

    if (old_v_s0 / v_lb_.value(s0) > (1 - lb_in_change_of_Vl_s0_)) {
      // Search got "stuck"
      std::cout << "Search Batch " << batch_id
                << " Improvement too small, stopping search and moving to TVI"
                << std::endl;
      break;
    }
    batch_id++;
  }

  std::cout << "[FTVI]: " << totalSuboptimalActions()
            << " pairs in ignore action" << std::endl;

  /*
   * The problem was not solved with simple search, so delegate the solution to
   * TVI using the SSP with pruned suboptimal actions. The main reason for this
   * is to increase the number of SCCs
   */
  PlannerTVI::solve(pruned_problem_, v_lb_, epsilon_, PlannerTVI::SccAlgorithm::KOSARAJU);
  solved_ = true;

  std::cout << "\nCSVHACK"
            << "," << gpt::__ppddl_domain_filename
            << "," << gpt::__ppddl_problem_filename
            << "," << gpt::seed
            << "," << gpt::algorithm
            << "," << gpt::heuristic
            << "," << v_lb_.value(ssp_.s0())
            << "," << get_cputime_usec()
            << "," << gpt::heur_ptr->totalCalls()
            << "," << gpt::total_computed_qvalues
            << "\n\n";
}



double PlannerFTVI::search(state_t const& s, HashsetState& visited) {
  // FWT: these variables are still static, despite the method not being
  // recursive anymore, because the search method might be invoke more than once
  // during the execution of the method solve.
  static VectorProbDistState v_pr(FTVI_INITIAL_NUM_PROB_DIST);
  static size_t max_depth = 16;
  double max_bellman_error = 0;

  class RecursionState {
   public:
    // current state
    state_t const& s;
    // flags if first call on s or a returning call
    bool returning_from_call;
    // keeps track of the loop over the resulting states
    ProbDistIface<state_t>::const_iterator ip;

    RecursionState(state_t const& sP, bool returning)
       : s(sP), returning_from_call(returning)
    { }
  };

  std::stack<RecursionState> recursion_state;
  recursion_state.push(RecursionState(s, false));
  visited.insert(s);

  fake_recurssion_loop:
  while (!recursion_state.empty()) {
    if (max_depth == recursion_state.size()) {
      std::cout << "Stack size reached depth " << recursion_state.size()
                << std::endl;
      max_depth = max_depth << 1;
    }
    RecursionState r = recursion_state.top();
    recursion_state.pop();
    size_t const depth = recursion_state.size();
    if (visited.size() % 500 == 0) {
      gpt::checkDeadline();
    }

    // Flags is the state r.s has a greedy action (i.e., not a dead end). This
    // will be overwritten given the value of r.returning_from_call
    bool has_a_min = true;

    if (! r.returning_from_call) {
      /*
       * First time seeing this fake recursion object
       */

      // If r.s is the goal, the this call is over and we can procedure with the
      // next call.
      if (pruned_problem_.isGoal(r.s)) {
        continue;
      }

      action_t const* a_greedy = Bellman::greedyAction(r.s, v_lb_,
                                                       pruned_problem_);
      if (a_greedy) {
        // r.s is not a dead end, therefore we will recurse on r.s and we need
        // to prepare v_pr and the iterator r.ip
        if (v_pr.size() == depth) {
          v_pr.emplace_back();
        }
        pruned_problem_.expand(*a_greedy, r.s, v_pr[depth]);
        r.ip = v_pr[depth].begin();
        has_a_min = true;
      }
      else {
        // There is no greedy action for r.s, thus we won't recurse and there is
        // no need to prepare v_pr and r.ip
        has_a_min = false;
      }
    }
    else {
      /*
       * Returning call
       */

      // All returning calls have a_min because a_min triggered the recursion
      has_a_min = true;
      // Since we're returning, the current recursion state has an iterator that
      // we need to continue processing and its current position was already
      // dealt with, so incrementing it.
      ++r.ip;
    }

    if (has_a_min) {
      for (; r.ip != v_pr[depth].end(); ++r.ip) {
        state_t const& s_prime = r.ip.event();
        if (visited.find(s_prime) == visited.end()) {
          visited.insert(s_prime);
          r.returning_from_call = true;
          recursion_state.push(r);
          // when x.returning_from_call == false, ip is ignored
          recursion_state.push(RecursionState(s_prime, false));
          goto fake_recurssion_loop;
        }
      }  // for each successor of (s, a_min)
    }  // if has_a_min, i.e., if a_min != NULL for r.s
    /*
     * else, i.e., if a_min == NULL, then s is a dead end and nothing needs
     * to be done since there is no descend to search and the bellman backup
     * should assign deadend_cost to this state.
     */

    // Done with the "recursive" calls of search for r.s, so we can compute its
    // Bellman residual.
    double cur_bellman_error = bellmanBackup(r.s);
    if (max_bellman_error < cur_bellman_error) {
      max_bellman_error = cur_bellman_error;
    }
  }  // while recursion_state stack is not empty
  return max_bellman_error;
}



double PlannerFTVI::bellmanBackup(state_t const& s) {
  double min_ub_q_value = gpt::dead_end_value.double_value();
  double min_lb_q_value = gpt::dead_end_value.double_value();
  double const v_ub_s = v_ub_.value(s);

  for (action_t const& a : pruned_problem_.applicableActions(s)) {
    double ub_q_value = pruned_problem_.cost(s,a).double_value();
    double lb_q_value = ub_q_value;

    /* FWT: doing the qValue manually here to save one call to SSPIface::expand.
     * The for loop bellow is equivalent to:
      q_ub = Bellman::qValue(s, a, v_ub_, pruned_problem_)
      q_lb = Bellman::qValue(s, a, v_lb_, pruned_problem_)
     */
    pruned_problem_.expand(a, s, pr_);
    for (auto const& ip : pr_) {
      state_t const& s_prime = ip.event();
      double prob_s_prime = ip.prob();
      lb_q_value += prob_s_prime * v_lb_.value(s_prime);
      ub_q_value += prob_s_prime * v_ub_.value(s_prime);
    }  // ip in result of a applied in s


    if (lb_q_value > v_ub_s) {
      // This action can be ignored since it is provenly suboptimal (action
      // elimination, see [Bertsekas, 2001])
      suboptimal_actions_[s].insert(&a);
    }
    else {
      // The update of the lower and upper bound must be in the else case
      // because they should take into account only the actions NOT being
      // ignored.
      if (lb_q_value < min_lb_q_value) { min_lb_q_value = lb_q_value; }
      if (ub_q_value < min_ub_q_value) { min_ub_q_value = ub_q_value; }
    }
  }  // for each action a \in A_pruned(s)
  double old_v_lb = v_lb_.value(s);
  v_lb_.get(s)->update(min_lb_q_value);
  v_ub_.get(s)->update(min_ub_q_value);

  return std::abs(old_v_lb - min_lb_q_value);
}
