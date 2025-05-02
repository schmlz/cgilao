#include <ostream>
#include <queue>

#include <boost/algorithm/string.hpp>

#include "planner_iface.h"
#include "cg-ilao-extended.h"
#include "policy_envelope_printer.h"

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ext/mgpt/states.h"
#include "../utils/die.h"

#include "../ext/det_planners/externalFFInterface.h"
#include "../ext/det_planners/externalLamaInterface.h"

/*******************************************************************************
 *
 * Partial SSP
 *
 ******************************************************************************/
PartialSSPExtended::PartialSSPExtended(SSPIface const& ssp)
    : ssp_(ssp), external_det_planner_ao_(nullptr), external_det_planner_mlo_(nullptr)
{
  partial_space_[ssp.s0()] = {};
  // Base case for the regression_partial_space_
  regression_partial_space_[ssp_.s0()] = {};
}

PartialSSPExtended::~PartialSSPExtended() {
  std::cout << "--> FF cputime usecs = " << ff_cputime_usecs_ << std::endl;
  statistics();
}


void PartialSSPExtended::insertColumn(state_t const& s, action_t const* a) {
  n_columns_added_ += 1;
  auto it = partial_space_.find(s);

  if (it == partial_space_.end() || it->second.size() == 0) {
    if (it == partial_space_.end()) {
      // Adding the empty vector because a will be added later
      partial_space_.insert(it, {s, {}});
      it = partial_space_.find(s);
    }

    // No need to check if a_prime is in partial_space_[s] because this is the first
    // time adding columns to s therefore no columns should be in it yet
    assert(it->second.size() == 0);

    // TODO(fwt): maybe move this to its own private function
    // First time adding a column to s, so we need to compute the regression columns
    static ProbDistStateHash pr_s_a;
    for (action_t const& a_prime : ssp_.applicableActions(s)) {
      if (&a_prime == a) { continue; }

      ssp_.expand(a_prime, s, pr_s_a);
      for (auto const& ip : pr_s_a) {
        state_t const& s_prime = ip.event();
        // this transition is s ---a_prime---> s_prime, thus we add to s_prime the
        // regression column (s, a_prime)
        regression_missing_cols_[s_prime].insert({s, &a_prime});
      }
    }
  }

  assert(it != partial_space_.end());
  VecActionPtr& hat_a_s = it->second;
  assert(std::find(hat_a_s.begin(), hat_a_s.end(), a) == hat_a_s.end());
  hat_a_s.push_back(a);

  if (a != nullptr) {
    // Adding to all s' s.t. P(s'|s,a) > 0 that (s,a) is a possible regression of s'
    static ProbDistStateHash pr_s_a;
    ssp_.expand(*a, s, pr_s_a);
    for (auto const& ip : pr_s_a) {
      state_t const& s_prime = ip.event();
      // this transition is s ---a_prime---> s_prime, thus we add to s_prime the
      // regression column (s, a_prime)
      regression_partial_space_[s_prime].insert({s, a});
    }
  }
}

void PartialSSPExtended::removeColumn(state_t const& s, action_t const* a) {
  // Column to remove should be inside partial SSP currently
  assert(partial_space_.find(s) != partial_space_.end());
  assert(std::find(partial_space_.at(s).begin(), partial_space_.at(s).end(), a)
         != partial_space_.at(s).end());

  // Remove from partial SSP
  auto& partial_actions = partial_space_[s];
  auto const it = std::find(partial_actions.begin(), partial_actions.end(), a);
  assert(it != partial_actions.end());
  partial_actions.erase(it);

  // CLEAR CACHES
  //
  // HACK: this is the issue with having caches, you need to make sure they're all up to date...
  external_space_.erase(s);

  // ASSUMPTION: at least one applicable action should remain
  //
  // If you want to break this assumption have to think carefully about whether this state becomes
  // fringe or dead end etc.
  assert(partial_actions.size() > 0);

  // Moving all regression info about (s, a) from internal to external
  static ProbDistStateHash pr_s_a;
  ssp_.expand(*a, s, pr_s_a);
  for (auto const& ip : pr_s_a) {
    state_t const& s_prime = ip.event();

    assert(regression_partial_space_.find(s_prime) != regression_partial_space_.end());
    regression_partial_space_.at(s_prime).erase({s, a});

    // Note: regression_missing_cols_ may not have an entry at s_prime yet, see insertColumn
    regression_missing_cols_[s_prime].insert({s, a});
  }
}

/*
 * Call this with a_ptr = ARGMIN_{a} Q(s, a)
 */
CGiLAOExtendedImprovementStatus PartialSSPExtended::updateV(state_t const& s, action_t const* a_ptr, const double q_s_a, hash_t& v, SetOfConstrs& cols_to_be_checked) {
  bool policy_changed = false;

  // If this fails, then we might miss the policy change
  assert(cur_policy_.find(s) != cur_policy_.end() || a_ptr != nullptr);
  if (cur_policy_[s] != a_ptr) {
    D(std::cout << "    policy changed for " << s.toStringFull(gpt::problem) << " from "
              << (cur_policy_[s] ? cur_policy_[s]->name() : "nullptr") << " to "
              << (a_ptr ? a_ptr->name() : "nullptr")
              << "  -- min_q_value = " << q_s_a << "\n");
    updateCurPolicy(s, a_ptr);
    policy_changed = true;
  }

  // Sometimes the only option from some states (in or out side the best policy) is to do loop
  // back and make an improper policy so this assert will fail.
  // assert(cur_policy_[s] != nullptr);

  double signed_residual = v.value(s) - q_s_a;
  if (signed_residual < -gpt::epsilon) {
    // V(s) will increase with this update
    assert(q_s_a > v.value(s));
    D(std::cout << "  V(s) increased from " << v.value(s) << " to " << q_s_a << std::endl);
    // if (isPartiallyExpanded(s)) {
    //   for (action_t const& a : ssp_.applicableActions(s)) {
    //     if (isEliminated(s, &a)) {
    //       continue;
    //     }
    //     if (!containsColumn(s, a)) {
    //       cols_to_be_checked.insert({s, &a});
    //     }
    //   }
    // }
    auto& external_actions_s = externalActions(s);
    for (auto it = external_actions_s.begin(); it < external_actions_s.end();) {
      action_t const* a = *it;
      // Lazy update: remove actions from external when they become internal
      if (containsColumn(s, *a) or isEliminated(s, a)) {
        it = external_actions_s.erase(it);
        continue;
      }
      // Add external actions to cols_to_be_checked
      else {
        cols_to_be_checked.insert({s, a});
        ++it;
      }
    }
  }
  else if (signed_residual > gpt::epsilon) {
    // V(s) will decrease with this update
    assert(q_s_a < v.value(s));
    D(std::cout << "  V(s) DEcreased from " << v.value(s) << " to " << q_s_a << std::endl);
    // Schedule external parents
    SetOfConstrs& regression_cols = regression_missing_cols_[s];
    for (auto it = regression_cols.begin(); it != regression_cols.end();) {
      StateActionPtr const& col = *it;
      assert(col.action_ptr != nullptr);
      if (containsColumn(col.state, *col.action_ptr)) {
        // Lazy prunning
        // TODO(fwt): see if this lazy prunning is actually saving time
        it = regression_cols.erase(it);
        continue;
      }

      if (isEliminated(col)) {
        // Similar pruning
        //
        // ASSUMPTION: columns are eliminated PERMANENTLY
        it = regression_cols.erase(it);
        continue;
      }

      cols_to_be_checked.insert(col);
      ++it;
    }
    // Schedule internal parents
    SetOfConstrs const& regression_cols_s = getInternalRegressionColumns(s);
    for (auto const& col : regression_cols_s) {
      assert(col.action_ptr != nullptr);
      cols_to_be_checked.insert(col);
    }
  }
  v.update(s, q_s_a);
  return {policy_changed, fabs(signed_residual)};
}

std::vector<state_t> PartialSSPExtended::getBestGoals(hash_t const& v, const size_t n_best_goals) const
{
  if (n_best_goals == 0) { return {}; }

  // Select candidates from partial SSP
  std::vector<state_t> candidate_goals;
  for (auto const& [s, s_actions] : partial_space_) {
    // Skip fringes, dead ends, and goals
    //
    // TODO(jsch): we may not want to skip fringes, need to double check! NOTE:
    // if we do include fringes, then we need to make sure that the state we are
    // replanning from is not a best goal!
    if (s_actions.empty() or s_actions[0] == nullptr) { continue; }
    assert(not ssp_.isGoal(s));

    candidate_goals.emplace_back(s);
  }

  // Sort candidate goals by V (low to high, i.e. from states "closest" to the
  // goal to states "furthest away" from the goal)
  auto candidateLessThan = [&](state_t const& a, state_t const& b) {
    return (v.value(a) < v.value(b));
  };
  std::sort(candidate_goals.begin(), candidate_goals.end(), candidateLessThan);

  // if (not candidate_goals.empty()) {
  //   std::cout << "-- candidate range **before** pruning: " <<
  //   v.value(candidate_goals.front())
  //             << " --- " << v.value(candidate_goals.back()) << "\n";
  // }

  // Pick out best goals
  if (candidate_goals.size() > n_best_goals) {
    candidate_goals.erase(candidate_goals.begin() + n_best_goals, candidate_goals.end());
  }

  // if (not candidate_goals.empty()) {
  //   std::cout << "-- candidate range **after** pruning: " <<
  //   v.value(candidate_goals.front())
  //             << " --- " << v.value(candidate_goals.back()) << "\n";
  // }

  return candidate_goals;
}

/*
 * Run trial from starting_s
 *
 * Returns SUCCESS if
 * - reaches goal
 * - reaches internal state
 *
 * Returns NO_PLAN if
 * - exceeds max number of steps
 * - reaches a dead end
 *
 * IMPORTANT: NO_PLAN does not mean that no plan exists, but rather that we do not have a
 * successful plan, so it doesn't necessarily make sense to add all its actions.
 */
DetPlannerReturnType PartialSSPExtended::runTrialFrom(
    state_t const& starting_s, std::vector<std::pair<double, state_t>>& state_trace,
    std::vector<action_t const*>& action_trace, hash_t& v, size_t const max_trial_steps)
{
  assert(!isInternal(starting_s));
  assert(!ssp_.isGoal(starting_s));

  state_trace.clear();
  action_trace.clear();
  state_t cur_s = starting_s;
  for (size_t trial_step = 0; trial_step < max_trial_steps; ++trial_step) {

    // Reached goal or internal state from which we "know what to do"
    if (ssp_.isGoal(cur_s) or
        (containsState(cur_s) and cur_s != starting_s)) {
      assert(cur_s != starting_s);
      // std::cout << "SUCCESS - goal or internal" << std::endl;
      return SUCCESS;
    }

    // std::cout << "trial step: " << trial_step
    //           << " state: " << cur_s.toStringFull(gpt::problem)
    //           << std::endl;

    assert(cur_s == starting_s or !containsState(cur_s));

    // Extract greedy action. If some of the actions lead to an internal state we find the greedy
    // action among those actions.
    //
    // NOTE: in this case we count a goal state as internal since we "know what to do" there
    action_t const* greedy_a = nullptr;
    bool greedy_a_leads_to_internal_state = false;
    double min_q_value = gpt::dead_end_value.double_value();

    for (auto const& a : ssp_.applicableActions(cur_s)) {
      static ProbDistState pr_s_a;
      ssp_.expand(a, cur_s, pr_s_a);

      // Compute Q(s,a) with framework and then check if a leads to internal state
      double q_value = Bellman::qValue(cur_s, a, v, ssp_);
      bool a_leads_to_internal = false;
      for (auto const& it : pr_s_a) {
        state_t const& s_prime = it.event();
        a_leads_to_internal |= ssp_.isGoal(s_prime) || isInternal(s_prime);
      }

      // Combine Q(s,a) computation with check for internal state -- don't want to do it this way
      // because it's a separate computation of qvalues which makes it a nightmare to track
      //
      // double q_value = ssp_.cost(cur_s, a).double_value();
      // bool a_leads_to_internal = false;
      // for (auto const& it : pr_s_a) {
      //   state_t const& s_prime = it.event();
      //   double const prob_s_prime = it.prob();
      //   q_value += prob_s_prime * v.value(s_prime);
      //   a_leads_to_internal |= ssp_.isGoal(s_prime) || isInternal(s_prime);
      // }

      if ((!greedy_a_leads_to_internal_state and a_leads_to_internal) or \
          (greedy_a_leads_to_internal_state == a_leads_to_internal and q_value < min_q_value)) {
        assert(a_leads_to_internal >= greedy_a_leads_to_internal_state);
        greedy_a = &a;
        min_q_value = q_value;
        greedy_a_leads_to_internal_state = a_leads_to_internal;
      }

    }

    // If greedy action leads to internal state we stop here
    if (greedy_a_leads_to_internal_state) {
      assert(greedy_a != nullptr);
      action_trace.emplace_back(greedy_a);
      greedy_a->affect(cur_s); // HACK(jsch): the final state DOES NOT MATTER but it should really be the reachable internal state...
      state_trace.emplace_back(-1.0, cur_s); // HACK(jsch): see runPlanFrom for info on left term
      // std::cout << "SUCCESS - a leads to internal" << std::endl;
      return SUCCESS;
    }

    // CAREFUL: doing an update to v here.
    //
    // This is necessary, otherwise the trial may get stuck in a loop. By updating V we've
    // effectively got an RTDP trial, which is guaranteed to stop eventually.
    //
    // IMPORTANT: if there is internal s' in supp(s, a) we have already returned and thus we **do
    //            not** update V(s) <- Q(s, a); required because internal states may not be
    //            admissible
    //
    // QUESTION: do updates to V on external states break things?
    // --> ONLY IF UPDATE ONLY LOOKS AT EXTERNAL STATES
    // --> policy is undefined, so no danger of updating policy
    // --> no need to track columns_to_be_checked
    // --> we are only looking at external states populated with h, so new V must remain admissible
    //
    // QUESTION: do updates to V(starting_s) break things?
    // --> NO.
    // --> currently does not have a policy defined (it's a fringe) so no danger of updating policy
    // --> we select the greedy action according to the most up-to-date V(starting_s), so we do not
    //     need to keep track of columns_to_be_checked yet.
    //
    // NOTE: the only states we encounter are starting_s (which is a fringe state) and external
    // states, no other states are possible!
    assert(!greedy_a_leads_to_internal_state);
    v.update(cur_s, min_q_value);

    // Dead end
    if (greedy_a == nullptr) {
      // std::cout << "NO_PLAN - dead end" << std::endl;
      return NO_PLAN;
    }

    action_trace.emplace_back(greedy_a);
    greedy_a->affect(cur_s);
    state_trace.emplace_back(-1.0, cur_s); // HACK(jsch): see runPlanFrom for info on left term
  }

  // Time-out: the number of trial steps has been exceeded
  //
  // std::cout << "NO_PLAN - timeout" << std::endl;
  return NO_PLAN;
}

DetPlannerReturnType PartialSSPExtended::runDetPlannerFrom(
    state_t const& s, std::vector<std::pair<double, state_t>>& state_trace,
    std::vector<action_t const*>& action_trace, const bool use_mlo,
    std::vector<state_t> const& extra_goals, const std::string det_planner)
{

  // Run most-likely-outcomes (MLO)
  if (use_mlo) {
    // Set up MLO planner if needed
    if (external_det_planner_mlo_ == nullptr) {
      external_det_planner_mlo_ = ExternalDetPlannerInterface::createInterface(
          *gpt::problem, (det_planner == "lama" ? 60 : 300), MOST_LIKELY_OUTCOMES, det_planner);
    }

    // Run MLO
    state_trace.clear();
    action_trace.clear();
    const auto mlo_retcode =
        external_det_planner_mlo_->planFrom(s, state_trace, &action_trace, 0.0, &extra_goals);

    if (mlo_retcode == SUCCESS) { return mlo_retcode; }
  }

  // Set up AO planner if needed
  if (external_det_planner_ao_ == nullptr) {
    external_det_planner_ao_ = ExternalDetPlannerInterface::createInterface(
        *gpt::problem, (det_planner == "lama" ? 60 : 300), ALL_OUTCOMES, det_planner);
  }

  // If we did not find a plan with MLO, run all-outcomes (AO)
  state_trace.clear();
  action_trace.clear();
  return external_det_planner_ao_->planFrom(s, state_trace, &action_trace, 0.0, &extra_goals);
}

void PartialSSPExtended::markDeadEnd(state_t const& s, hash_t& v) {
    v.update(s, gpt::dead_end_value.double_value());
    updateCurPolicy(s, nullptr);
    assert(partial_space_.find(s) == partial_space_.end() || partial_space_.find(s)->second.size() == 0);
    // Note: by inserting an action (nullptr in this case) for s we are turning s into a non-fringe state
    insertColumn(s, nullptr);
    assert(!isFringe(s));
}

// NOTE: reachable_non_fringes does **not** include all reachable non-fringes, but rather the
// reachable non-fringes that are not within the state_trace with an action
void PartialSSPExtended::markReachableFringesAndNonFringes(state_t const& starting_s,
                                      std::vector<std::pair<double, state_t>> const& state_trace,
                                      std::vector<action_t const*> const& action_trace,
                                      SetOfStates& reachable_fringes,
                                      SetOfStates& reachable_non_fringes)
{
  // state_t cur_s = starting_s;

  assert(state_trace.size() == action_trace.size() + 1);
  assert(state_trace.size() > 0);

  for (size_t i = 0; i < action_trace.size(); ++i) {
    state_t cur_s = state_trace[i].second;
    action_t const* a = action_trace[i];
    assert(a != nullptr);

    // // Don't mark states within state_trace
    // //
    // // NOTE: we **do** still mark the final state in the trace
    // reachable_non_fringes.erase(cur_s);

    static ProbDistStateHash pr_s_a;
    ssp_.expand(*a, cur_s, pr_s_a);
    for (auto const& ip : pr_s_a) {
      state_t const& s_prime = ip.event();

      // This call will add an empty vector if s_prime is not in partial_space_ which is intended!
      if (partial_space_[s_prime].size() > 0 || ssp_.isGoal(s_prime)) {
        // s_prime is NOT a fringe state
        reachable_non_fringes.insert(s_prime);
        continue;
      }
      assert(isFringe(s_prime));
      reachable_fringes.insert(s_prime);
    }
    // std::cout << "next state reachable: " << next_state_reachable << std::endl;

    // // Updating cur_s. See planFrom documentation for explanation
    // cur_s = state_trace[i].second;
  }
}

// TODO(jsch): can probably refactor this with expandFringesWithFF
void PartialSSPExtended::expandFringesWithTrial(state_t const& s, hash_t& v, SetOfStates& new_fringes, SetOfStates& reached_non_fringe, size_t const max_trial_steps)
{
  assert(isFringe(s));

  // TODO(fwt) make sure that goals are never in the fringes... for now just skipping it
  assert(!ssp_.isGoal(s));

  static DetPlannerReturnType retcode;
  static std::vector<std::pair<double, state_t>> state_trace;
  static std::vector<action_t const*> action_trace;

  retcode = runTrialFrom(s, state_trace, action_trace, v, max_trial_steps);

  assert(retcode != TIMEOUT);

  // If the trial "failed", i.e., it did not reach a goal or internal state, then we just fall back
  // on Bellman expansion.
  //
  // NOTE: the trial may have made some changes to V, making the Bellman expansion more informed,
  // so it wasn't necessarily a waste of time
  if (retcode == NO_PLAN) {
    expandFringeWithBellman(s, v, CGiLAOExtendedExpansionType::bellman_tied,
                            new_fringes, &reached_non_fringe);
    return;
  }

  assert(state_trace.size() == action_trace.size());
  assert(state_trace.size() > 0);
  assert(retcode == SUCCESS);

  // prepend s to state_trace
  state_trace.insert(state_trace.begin(), std::make_pair(1.0, s));

  // Remove any loops from the trial
  for (size_t i = 0; i < state_trace.size()-1; ++i) {
    state_t s_i = state_trace[i].second;

    // find last occurrence of s_i
    size_t last_occurrence = i;
    for (size_t j = 0; j < state_trace.size()-1; j++) {
      if (state_trace[j].second == s_i) {
        last_occurrence = j;
      }
    }

    // if last occurrence is not at i we remove all states and actions from i to last occurrence
    if (i != last_occurrence) {
      state_trace.erase(state_trace.begin() + i, state_trace.begin() + last_occurrence);
      action_trace.erase(action_trace.begin() + i, action_trace.begin() + last_occurrence);
    }
  }

#ifndef NDEBUG
  for (size_t i = 0; i < action_trace.size(); ++i) {
    assert(ssp_.isApplicable(state_trace[i].second, *action_trace[i]));
  }
#endif

  for (size_t i = 0; i < action_trace.size(); ++i) {

    state_t cur_s = state_trace[i].second;

    // TODO: This seems better and checking if s_prime != state_trace[i] and also affect different
    // runs of FF
    new_fringes.erase(cur_s);

    // auto it = partial_space_.find(cur_s);
    // if (it != partial_space_.end() && it->second.size() > 0) {
    //   // We already have an action for it so skipping it
    //   // std::cout << "---> Stopping early because " << cur_s.toStringFull(gpt::problem) << " is not a fringe\n";
    //   reached_non_fringe.insert(cur_s);
    //   break;
    // }

    action_t const* a = action_trace[i];
    assert(a != nullptr);

    // assert(it == partial_space_.end() || it->second.size() == 0);

    // Trail may have loop, so column may have already been added!
    if (!containsColumn(cur_s, *a)) {
      insertColumn(cur_s, a);
    }

    assert(!isFringe(cur_s));

    // Trial may have loops, so we need to make sure to override the policy to the latest entry
    updateCurPolicy(cur_s, a);

    // // Updating cur_s. See planFrom documentation for explanation
    // cur_s = state_trace[i].second;
  }

  // Make sure to run this **after** state+action trace have already been expanded
  markReachableFringesAndNonFringes(s, state_trace, action_trace, new_fringes, reached_non_fringe);

}

void PartialSSPExtended::expandFringesWithFF(state_t const& s, hash_t& v, SetOfStates& new_fringes,
                                     const bool use_mlo, std::vector<state_t> const& extra_goals,
                                     SetOfStates& reached_non_fringe)
{
  assert(isFringe(s));

  // TODO(fwt) make sure that goals are never in the fringes... for now just skipping it
  assert(!ssp_.isGoal(s));

  static DetPlannerReturnType retcode;
  static std::vector<std::pair<double, state_t>> state_trace;
  static std::vector<action_t const*> action_trace;

uint64_t const before = get_cputime_usec();
  retcode = runDetPlannerFrom(s, state_trace, action_trace, use_mlo, extra_goals);
uint64_t const after = get_cputime_usec();
ff_cputime_usecs_ += (after - before);
  gpt::checkDeadline();

  assert(retcode != TIMEOUT);

  if (retcode == NO_PLAN) {
    markDeadEnd(s, v);
    return;
  }

  assert(retcode == SUCCESS);

  state_t cur_s = s;

  // std::cout << "FF plan:\n";
  // std::cout << "  " << s.toStringFull(gpt::problem) << ", ";
  // for (size_t i = 0; i < action_trace.size(); ++i) {
  //   std::cout << "  a =" << action_trace[i]->name()
  //             << "\n  " << state_trace[i].second.toStringFull(gpt::problem);
  // }
  // std::cout << (ssp_.isGoal(state_trace[state_trace.size()-1].second) ? " goal\n" : " **NOT** goal\n");

  for (size_t i = 0; i < action_trace.size(); ++i) {
    // TODO: This seems better and checking if s_prime != state_trace[i] and also affect different
    // runs of FF
    new_fringes.erase(cur_s);

    auto it = partial_space_.find(cur_s);
    if (it != partial_space_.end() && it->second.size() > 0) {
      // We already have an action for it so skipping it
      // std::cout << "---> Stopping early because " << cur_s.toStringFull(gpt::problem) << " is not a fringe\n";
      reached_non_fringe.insert(cur_s);
      break;
    }

    action_t const* a = action_trace[i];
    assert(a != nullptr);

    assert(it == partial_space_.end() || it->second.size() == 0);
    insertColumn(cur_s, a);

    // Since we did not have any action considered for s, the pi(s) must NOT be defined
    assert(cur_policy_.find(cur_s) == cur_policy_.end());
    updateCurPolicy(cur_s, a);

    static ProbDistStateHash pr_s_a;
    ssp_.expand(*a, cur_s, pr_s_a);
    for (auto const& ip : pr_s_a) {
      state_t const& s_prime = ip.event();
      // This call will add an empty vector if s_prime is not in partial_space_ which is intended!
      if (partial_space_[s_prime].size() > 0 || ssp_.isGoal(s_prime)) {
        // s_prime is NOT a fringe state
        reached_non_fringe.insert(s_prime);
        continue;
      }
      new_fringes.insert(s_prime);
    }
    // Updating cur_s. See planFrom documentation for explanation
    cur_s = state_trace[i].second;
  }
}


// TODO: this should probably be factored into Bellman::
double extendedGreedyActionAndMinQValueWithTies(state_t const& s, hash_t& hash, SSPIface const& ssp,
    std::vector<action_t const*>& tied_greedy_actions, double epsilon = 0.0)
{
  tied_greedy_actions.clear();
  tied_greedy_actions.emplace_back(nullptr);

  if (ssp.isGoal(s)) {
    return ssp.terminalCost(s).double_value();
  }

  double min_q_value = gpt::dead_end_value.double_value();
  for (auto const& a : ssp.applicableActions(s)) {
    double const q_s_a = Bellman::qValue(s, a, hash, ssp);
    if (q_s_a < min_q_value) {
      min_q_value = q_s_a;
      tied_greedy_actions.clear();
      tied_greedy_actions.emplace_back(&a);
    } else if (q_s_a == min_q_value) {
      tied_greedy_actions.emplace_back(&a);
    }
  }

  return min_q_value;
}

void PartialSSPExtended::expandFringeWithBellman(state_t const& s, hash_t& v,
    CGiLAOExtendedExpansionType const& expansion_type, SetOfStates& new_fringes, SetOfStates* reached_non_fringe)
{
  assert(expansion_set_up_);  // HACK(jsch): make sure expansion has been set up properly

  // Method should only be called on true fringe states
  assert(isFringe(s));

  // TODO(fwt) make sure that goals are never in the fringes... for now just skipping it
  assert(!ssp_.isGoal(s));


  D(std::cout << "    [expandFringeWithBellman] expanding s = "
              << s.toStringFull(gpt::problem) << std::endl;);

  action_t const* greedy_action = nullptr;
  double min_q_value = -1;
  std::vector<action_t const*> columns_from_s_to_add;

  if (expansion_type == CGiLAOExtendedExpansionType::bellman) {
    std::tie(greedy_action, min_q_value) = Bellman::greedyActionAndMinQValue(s, v, ssp_);
    columns_from_s_to_add.push_back(greedy_action);
  }
  else if (expansion_type == CGiLAOExtendedExpansionType::bellman_tied) {
    min_q_value = extendedGreedyActionAndMinQValueWithTies(s, v, ssp_, columns_from_s_to_add, gpt::epsilon);
    assert(columns_from_s_to_add.size() > 0);
    greedy_action = columns_from_s_to_add[0];
  }
  else if (expansion_type == CGiLAOExtendedExpansionType::bellman_n_best) {
    // Find n cheapest unique Q-values, and add all actions with those Q-values
    std::vector<double> unique_qvalues;
    std::unordered_map<double, std::vector<action_t const*>> qvalues_to_actions;
    for (action_t const& a : ssp_.applicableActions(s)) {
      const double q_s_a = Bellman::qValue(s, a, v, ssp_);
      if (std::find(unique_qvalues.begin(), unique_qvalues.end(), q_s_a) == unique_qvalues.end()) {
        unique_qvalues.emplace_back(q_s_a);
      }
      qvalues_to_actions[q_s_a].emplace_back(&a);
      assert(qvalues_to_actions.find(q_s_a) != qvalues_to_actions.end());
    }

    max_unique_qvalues_ = std::max(max_unique_qvalues_, unique_qvalues.size());

    if (unique_qvalues.size() > 0) {
      std::sort(unique_qvalues.begin(), unique_qvalues.end());
      min_q_value = unique_qvalues[0];
      greedy_action = qvalues_to_actions.at(min_q_value)[0];
      for (size_t i = 0; i < expansion_n_ and i < unique_qvalues.size(); ++i) {
        const double qval_i = unique_qvalues[i];
        std::vector<action_t const*>& acts = qvalues_to_actions.at(qval_i);
        // columns_from_s_to_add.insert(columns_from_s_to_add.end(), acts.begin(), acts.end());
        for (action_t const* a_ptr : acts) { columns_from_s_to_add.emplace_back(a_ptr); }
      }
    }
  }
  else if (expansion_type == CGiLAOExtendedExpansionType::bellman_x_best) {
    // Add actions with sufficiently small Q-values
    std::vector<std::pair<action_t const*, double>> qvalues;
    double min_qvalue = gpt::dead_end_value.double_value();
    double max_qvalue = -1.0;
    for (action_t const& a : ssp_.applicableActions(s)) {
      const double q_s_a = Bellman::qValue(s, a, v, ssp_);
      min_qvalue = std::min(min_qvalue, q_s_a);
      max_qvalue = std::max(max_qvalue, q_s_a);
      qvalues.emplace_back(&a, q_s_a);
    }

    const double max_allowed_qvalue = min_qvalue + expansion_x_ * (max_qvalue - min_qvalue);
    min_q_value = min_qvalue;
    for (auto const& [a_ptr, q_s_a] : qvalues) {
      if (q_s_a <= max_allowed_qvalue) {
        columns_from_s_to_add.emplace_back(a_ptr);

        if (q_s_a <= min_q_value) {
          greedy_action = a_ptr;
        }
      }
    }
  }
  else if (expansion_type == CGiLAOExtendedExpansionType::bellman_x2_best) {
    // Add x cheapest Q-values

    // NOTE: multimap sorts elements
    std::multimap<double, action_t const*> qvalues;
    for (action_t const& a : ssp_.applicableActions(s)) {
      const double q_s_a = Bellman::qValue(s, a, v, ssp_);
      qvalues.emplace(q_s_a, &a);
    }

    if (!qvalues.empty()) {
      assert(qvalues.begin()->first <= (--qvalues.end())->first);
      greedy_action = qvalues.begin()->second;
      min_q_value = qvalues.begin()->first;

      // Make sure at least one action is added
      const size_t n_actions_to_add = std::max(size_t(1), size_t(std::ceil(expansion_x_ * double(qvalues.size()))));
      assert(n_actions_to_add > 0);
      assert(n_actions_to_add <= qvalues.size());
      // std::cout << "====" << n_actions_to_add << std::endl;
      for (size_t n = 0; n < n_actions_to_add; ++n) {
        const auto it = qvalues.extract(qvalues.begin());
        // std::cout << "---" << it.key() << std::endl;
        action_t const* a_ptr = it.mapped();
        columns_from_s_to_add.emplace_back(a_ptr);
      }
      assert(columns_from_s_to_add.size() == n_actions_to_add);
    }

  }
  else if (expansion_type == CGiLAOExtendedExpansionType::bellman_x3_best) {
    // Add actions with Qvalues <= x * min_a Q(s,a)
    //
    // ASSUMPTION: x >= 1
    assert(expansion_x_ >= 1.0);

    // NOTE: multimap sorts elements
    std::multimap<double, action_t const*> qvalues;
    for (action_t const& a : ssp_.applicableActions(s)) {
      const double q_s_a = Bellman::qValue(s, a, v, ssp_);
      qvalues.emplace(q_s_a, &a);
    }

    if (!qvalues.empty()) {
      assert(qvalues.begin()->first <= (--qvalues.end())->first);
      min_q_value = qvalues.begin()->first;
      greedy_action = qvalues.begin()->second;
      // add all actions with q_s_a <= expansion_x_ * min_q_value
      for (auto it = qvalues.begin(); it != qvalues.end(); ++it) {
        if (it->first <= expansion_x_ * min_q_value) {
          columns_from_s_to_add.emplace_back(it->second);
        } else {
          break;
        }
      }
    }

  }
  else {
    assert(expansion_type == CGiLAOExtendedExpansionType::complete);
    std::tie(greedy_action, min_q_value) = Bellman::greedyActionAndMinQValue(s, v, ssp_);
    for (action_t const& a : ssp_.applicableActions(s)) {
      columns_from_s_to_add.push_back(&a);
    }
  }

  if (greedy_action == nullptr) {
    markDeadEnd(s, v);
    return;
  }

  // Since we did not have any action considered for s, the pi(s) must NOT be defined
  assert(cur_policy_.find(s) == cur_policy_.end());
  updateCurPolicy(s, greedy_action);

  assert(std::find(columns_from_s_to_add.begin(), columns_from_s_to_add.end(), greedy_action)
          != columns_from_s_to_add.end());

  // Inserting into partial space all the columns (s,a) for a in columns_from_s_to_add. Only the
  // greedy action will be expanded, i.e., its successors will be considered as either new fringe
  // states or reachable non-fringe. For complete, this is equivalent to iLAO*, all actions are
  // added to the partial_space and the greedy action is expanded
  static ProbDistStateHash pr_s_a;
  for (action_t const* a : columns_from_s_to_add) {
    assert(a != nullptr);
    insertColumn(s, a);
    ssp_.expand(*a, s, pr_s_a);
    for (auto const& ip : pr_s_a) {
      state_t const& s_prime = ip.event();
      if (ssp_.isGoal(s_prime)) { continue; }

      auto it = partial_space_.find(s_prime);
      if (it == partial_space_.end()) {
        // Insert an entry with the empty vector and updates the iterator
        it = partial_space_.insert(it, {s_prime, {}});
      }

      // Since a is not the greedy action, i.e., not in the current best policy, we don't need to
      // process its decendents
      if (a != greedy_action) { continue; }

      if (it->second.size() > 0) {
        // s_prime is NOT a fringe state
        D(std::cout << "    [expandFringeWithBellman] reachable state is NOT a fringe: "
                    << s_prime.toStringFull(gpt::problem) << std::endl;);
        if (reached_non_fringe != nullptr) {
          reached_non_fringe->insert(s_prime);
        }
      }
      else {
        D(std::cout << "    [expandFringeWithBellman] new fringe: "
                    << s_prime.toStringFull(gpt::problem) << std::endl;);
        // FWT: s_prime is added to new_fringe because either
        //  (1) partial_space_ did not have s_prime OR
        //  (2) partial_space_[s_prime].size() == 0
        // In the definition of isFringe, (2) qualifies as s_prime as a fringe, so technically a
        // current fringe state can be potentially added to new_fringe.
        //
        // This is OK because new_fringes tracks a *superset* of the true fringes of the new current
        // policy. So, having a state that soon will stop being a fringe is fine and the code will
        // handle it.
        new_fringes.insert(s_prime);
      }
    }
  }
}


void PartialSSPExtended::dfsSimilationOfCurPolicy(state_t const& s, hash_t& v, SetOfStates& new_fringes,
    ListOfStates& postorder_traversal, bool expand_fringe_states,
    CGiLAOExtendedExpansionType const& expansion_type)
{
  static SetOfStates open_or_closed;

#ifndef NDEBUG
  // Find all the current fringes before the DFS does any expansion -- this is used after DFS
  //
  // EXPENSIVE: we're doing another DFS!
  postorder_traversal.clear();
  new_fringes.clear();
  open_or_closed.clear();
  open_or_closed.insert(s);
  dfsSimilationOfCurPolicyRec(s, v, new_fringes, 0, open_or_closed, postorder_traversal,
                              false, expansion_type);
  const SetOfStates old_fringes_DEBUG(new_fringes);
#endif

  postorder_traversal.clear();
  new_fringes.clear();
  open_or_closed.clear();
  open_or_closed.insert(s);
  dfsSimilationOfCurPolicyRec(s, v, new_fringes, 0, open_or_closed, postorder_traversal,
                              expand_fringe_states, expansion_type);

#ifndef NDEBUG
  // Make sure that new_fringes are indeed fringes
  for (state_t const& f : new_fringes) { assert(isFringe(f)); }

  // If expanding: make sure that old_fringes are no longer fringes
  if (expand_fringe_states) {
    for (state_t const& f : old_fringes_DEBUG) { assert(not isFringe(f)); }
  }
#endif
}


// TODO(fwt): Adapted from ilao.cc. Generalize and removed duplicated code
void PartialSSPExtended::dfsSimilationOfCurPolicyRec(state_t const& s, hash_t& v, SetOfStates& new_fringes,
    size_t depth, SetOfStates& open_or_closed, ListOfStates& postorder_traversal,
    bool expand_fringe_states, CGiLAOExtendedExpansionType const& expansion_type)
{
  D(std::cout << "[partialSSP::dfsSimilationOfCurPolicyRec] Depth " << depth << " state = "
      << s.toStringFull(gpt::problem) << std::endl);

  // static size_t deadline_counter = 0;
  // gpt::incCounterAndCheckDeadlineEvery(deadline_counter, 500);
  static size_t max_depth_seen_so_far = 1;
  if (max_depth_seen_so_far == depth) {
    std::cout << "DFS: Reached depth " << depth << std::endl;
    max_depth_seen_so_far = max_depth_seen_so_far << 1;
  }

  if (v_pr_.size() == depth) {
    v_pr_.emplace_back();
  }

  // We don't need to include the goal since a Bellman backup of the goal is useless
  if (ssp_.isGoal(s)) {
    if (!has_seen_goal_ and action_elim_enabled_) {
      std::cout << "[cg-ilao::dfs] found first goal, starting action elimination" << std::endl;
      // Set up upper bound heuristic and value function
      h_ub_.emplace(ssp_, gpt::dead_end_value.double_value());
      v_ub_.emplace(gpt::initial_hash_size, *h_ub_);
    }
    has_seen_goal_ = true;
    D(std::cout << "  - Goal. Returning\n");
    // Goals are not included in the post-order traversal
    return;
  }

  // If we encounter a "new fringe" state s via a path without previously closed states, then s was
  // already in the DFS envelope, and therefore is **not** a new fringe.
  if (new_fringes.find(s) != new_fringes.end()) {
    new_fringes.erase(s);
  }

  if (isInternal(s)) {
    assert(cur_policy_.find(s) != cur_policy_.end());
    action_t const* pi_s = cur_policy_[s];

    if (pi_s == nullptr) {
      D(std::cout << "  - pi_s == nullptr. Returning\n");
      // Dead ends are not included in the post-order traversal
      return;
    }

    // FWT: Be careful! pr_ cannot be reused in the recursion!
    assert(pi_s != nullptr);
    D(std::cout << "  - Internal State. Continuing by applying cur_pi(s) = " << pi_s->name() << "\n");
    ssp_.expand(*pi_s, s, v_pr_[depth]);
    for (auto const& ip : v_pr_[depth]) {
      state_t const& s_prime = ip.event();
      if (open_or_closed.find(s_prime) != open_or_closed.end()) { continue; }

      D(std::cout << "  recursing on " << s_prime.toStringFull(gpt::problem) << std::endl);
      open_or_closed.insert(s_prime);
      dfsSimilationOfCurPolicyRec(s_prime, v, new_fringes, depth+1, open_or_closed,
                                  postorder_traversal, expand_fringe_states, expansion_type);
    }
  }
  else if (isFringe(s)) {
    if (!expand_fringe_states) {
      D(std::cout << "  - isFringe. Stopping\n");

      // Reinsert fringe -- this is only necessary if we're not expanding it
      new_fringes.insert(s);

      // Fringe states are not included in the post-order traversal
      return;
    }
    D(std::cout << "  - isFringe. Expanding it\n");
    SetOfStates non_fringe_states_reached;
    expandFringeWithBellman(s, v, expansion_type, new_fringes, &non_fringe_states_reached);
    for (state_t const& s_prime : non_fringe_states_reached) {
      assert(!isFringe(s_prime));
      assert(cur_policy_.find(s_prime) != cur_policy_.end());
      if (open_or_closed.find(s_prime) != open_or_closed.end()) { continue; }

      D(std::cout << "  recursing on NON-FRINGE expansion of FRINGE "
                  << s_prime.toStringFull(gpt::problem) << std::endl);
      open_or_closed.insert(s_prime);
      dfsSimilationOfCurPolicyRec(s_prime, v, new_fringes, depth+1, open_or_closed,
                                  postorder_traversal, expand_fringe_states, expansion_type);
    }
  }
  else {
    // This should not happen
    D(std::cout << "  - This should not be reachable\n");
    NOT_IMPLEMENTED;
  }
  postorder_traversal.push_back(s);
}



// An internal expansion is to add a column (s,a) when s \in hat(S) and \hat{A}(s) != emptyset,
// i.e., s is already in the partial SSP and there is at least one applicable action for s.
//
// IMPORTANT: a partial expansion can reach states outside hat(S), i.e., there is not restriction or
// guarantee regarding the states s' s.t. P(s'|s,a) > 0.
void PartialSSPExtended::expandInternalStates(SetOfConstrs const& columns_to_be_added) {
  for (StateActionPtr const& col : columns_to_be_added) {
    assert(isPartiallyExpanded(col.state));
    assert(!containsColumn(col.state, *col.action_ptr));

    insertColumn(col.state, col.action_ptr);

    // The expansion of a partially expanded states can reach unseen states
    static ProbDistStateHash pr_s_a;
    ssp_.expand(*col.action_ptr, col.state, pr_s_a);
    for (auto const& ip : pr_s_a) {
      state_t const& s_prime = ip.event();
      if (ssp_.isGoal(s_prime)) { continue; }
      auto ik = partial_space_.find(s_prime);
      if (ik == partial_space_.end()) {
        // std::cout << "partial_space_.insert(" << s_prime.toStringFull(gpt::problem) << ")\n";
        // inserting the empty vector
        partial_space_.insert(ik, {s_prime, {}});
      }
    }
  }
}


CGiLAOExtendedImprovementStatus PartialSSPExtended::valueIterationPartialSSPExtended(hash_t& v, double epsilon, SetOfConstrs& cols_to_be_checked) {
  double max_residual = 1.0 + epsilon;
  bool pi_updated = false;
  // size_t iter = 0;

  while (max_residual > epsilon) {
    max_residual = 0;
    // iter++;
    for (auto const& pair : partial_space_) {
      state_t const& s = pair.first;
      if (pair.second.size() == 0) {
        // fringe state
        continue;
      }

      // CAREFUL: HACK: never tracks column age
      // (that's the last "false" parameters)
      CGiLAOExtendedImprovementStatus status = partialBellmanBackup(s, v, cols_to_be_checked,
                                                                    false);

      max_residual = std::max(max_residual, status.max_residual);
      pi_updated |= status.stopped_due_policy_change;
    }
  }
  // std::cout << "[vi] done. Number of iterations: " << iter << std::endl;
// #ifdef VI_SHOW_SOLUTION
//   std::cout << std::endl << "[vi::solution]";
//   for (state_t const& s : reachable_states) {
//     std::cout << "\t";
//     s.full_print(std::cout, gpt::problem);
//
//     double min_q_value = 0;
//     action_t const* a_greedy = nullptr;
//     std::tie(a_greedy, min_q_value) =
//                                  Bellman::greedyActionAndMinQValue(s, v, ssp);
//     std::cout << " -- V* = " << min_q_value
//               << "  pi* = " << (a_greedy ? a_greedy->name() : "NULL")
//               << std::endl;
//   }
// #endif
  return {pi_updated, max_residual};
}


/*
 * TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
 *
 * Incorporate this function to the rest of the code to remove the horrible duplication everywhere
 */
CGiLAOExtendedImprovementStatus PartialSSPExtended::partialBellmanBackup(state_t const& s, hash_t& v,
    SetOfConstrs& cols_to_be_checked, bool const track_age)
{
  // TODO: move pop to the bottom and make s a const ref
  D(std::cout << "Partial Bellman Backup for " << s.toStringFull(gpt::problem)
              << " -- V(s) = " << v.value(s) << std::endl);

  auto const it = partial_space_.find(s);
  assert(it != partial_space_.end());

  auto const& available_actions = it->second;

  assert(!ssp_.isGoal(s));
  // if (ssp_.isGoal(s)) {
  //   assert(ssp_.terminalCost(s) == 0);
  //   continue;
  // }

  assert(available_actions.size() > 0);

  if (available_actions.size() == 1 && available_actions[0] == nullptr) {
    // This is a state marked as dead end. Nothing else to do here
    assert(v.value(s) == gpt::dead_end_value.double_value());
    assert(cur_policy_.find(s) != cur_policy_.end());
    assert(cur_policy_.find(s)->second == nullptr);
    return {false, 0};
  }

  double min_q_value = gpt::dead_end_value.double_value();
  action_t const* argmin_q_value = nullptr;

  // FIXME: code duplication
  //
  // handles case where we DO track the age of stale columns
  if (track_age) {
    std::vector<std::pair<action_t const*, double>> q_values;

    for (action_t const* a : available_actions) {
      assert(a != nullptr);
      assert(ssp_.isApplicable(s, *a));
      double q_s_a = Bellman::qValue(s, *a, v, ssp_);
      q_values.emplace_back(a, q_s_a);
      D(std::cout << "  Q(" << s.toStringFull(gpt::problem) << ", " << a->name() << ") = " << q_s_a
                << std::endl);
      if (q_s_a < min_q_value) {
        argmin_q_value = a;
        min_q_value = q_s_a;
      }
    }

    for (auto const& [a, q_s_a] : q_values) {
      if (q_s_a > min_q_value + gpt::epsilon) {
        column_ages_[{s, a}] += 1;
      } else {
        column_ages_[{s, a}] = 0;
      }
    }

    return updateV(s, argmin_q_value, min_q_value, v, cols_to_be_checked);
  }

  // FIXME: code duplication
  //
  // handles case where we DO action elimination
  if (action_elim_enabled_ and has_seen_goal_) {
    std::vector<action_t const*> actions_eliminated_now;
    double min_q_value_ub = gpt::dead_end_value.double_value();

    for (action_t const* a : available_actions) {
      assert(a != nullptr);
      assert(ssp_.isApplicable(s, *a));
      double q_s_a = Bellman::qValue(s, *a, v, ssp_);

      // Eliminate Action
      //
      // NOTE: we require v_lb to be a lower bound, otherwise action elimination breaks
      if (cols_to_be_checked.empty() and q_s_a > v_ub_->value(s)) {
        actions_eliminated_now.emplace_back(a);
      }

      // Update action (if it wasn't eliminated)
      else {
        if (q_s_a < min_q_value) {
          argmin_q_value = a;
          min_q_value = q_s_a;
        }

        double q_s_a_ub = Bellman::qValue(s, *a, *v_ub_, ssp_);
        if (q_s_a_ub < min_q_value_ub) {
          min_q_value_ub = q_s_a_ub;
        }
      }
    }

    // note: v_ub_ must be monotonically non-increasing
    v_ub_residual_ = std::max(v_ub_residual_,
                              v_ub_.value().value(s) - min_q_value_ub);

    v_ub_->update(s, min_q_value_ub);

    // Process eliminated actions
    for (action_t const* a : actions_eliminated_now) {
      eliminateColumn(s, a, cols_to_be_checked);
    }

    return updateV(s, argmin_q_value, min_q_value, v, cols_to_be_checked);
  }

  for (action_t const* a : available_actions) {
    assert(a != nullptr);
    assert(ssp_.isApplicable(s, *a));
    double q_s_a = Bellman::qValue(s, *a, v, ssp_);
    D(std::cout << "  Q(" << s.toStringFull(gpt::problem) << ", " << a->name() << ") = " << q_s_a
              << std::endl);
    if (q_s_a < min_q_value) {
      argmin_q_value = a;
      min_q_value = q_s_a;
    }
  }

  return updateV(s, argmin_q_value, min_q_value, v, cols_to_be_checked);
}

size_t PartialSSPExtended::removeStaleColumns(int const max_age) {
  if (max_age < 0) {
    return 0;
  }

#ifndef NDEBUG
  size_t size_before = column_ages_.size();
#endif

  size_t n_cols_removed = 0;

  // Iterate over column_ages_, remove columns that are stale, and remove them from the age map
  for (auto it = column_ages_.begin(); it != column_ages_.end();) {
    auto const& [col, col_age] = *it;
    if (col_age > max_age) {
      removeColumn(col.state, col.action_ptr);
      ++n_cols_removed;
      it = column_ages_.erase(it);
    } else {
      it++;
    }
  }

#ifndef NDEBUG
  assert(size_before - column_ages_.size() == n_cols_removed);
#endif

  return n_cols_removed;
}

void PartialSSPExtended::dump(hash_t const* v) const {
  std::cout << "==== PartialSSPExtended::dump() ====\n";
  for (auto const& pair : partial_space_) {
    state_t const& s = pair.first;
    auto const& considered_actions = pair.second;
    size_t num_applicable_actions = 0;
    for ([[maybe_unused]] action_t const& a : ssp_.applicableActions(s)) {
      num_applicable_actions++;
    }
    std::cout << s.toStringFull(gpt::problem);
    if (v != nullptr) {
      std::cout << " -- h(s) = " << v->getHeuristic()->value(s)
                << " V(s) = " << v->value(s);
    }
    std::cout << " --  total |A| = " << num_applicable_actions
              << " |hat{A}| = " << considered_actions.size() << ":\n";
    for (action_t const* a : considered_actions) {
      std::cout << "  - " << (a ? a->name() : "nullptr");
      if (a != nullptr && v != nullptr) {
        std::cout << "  Q(s,a) = " << Bellman::constQValue(s, *a, *v, ssp_);
      }
      std::cout << "\n";
    }
  }
}

void PartialSSPExtended::printPartialSSPSize() const {
  size_t n_internal_states = 0;
  size_t n_fringe_and_goal_states = 0;
  size_t n_other_states = 0;
  size_t n_actions = 0;
  size_t n_actions_no_nullptr = 0;
  size_t n_applicable_actions = 0;

  for (auto const& [s, s_partial_actions] : partial_space_) {
    if (ssp_.isGoal(s) or isFringe(s)) {
      n_fringe_and_goal_states += 1;
    } else if (isInternal(s)) {
      n_internal_states += 1;
      n_actions += partial_space_.at(s).size();
      for (auto const* a_ptr : partial_space_.at(s)) {
        if (a_ptr == nullptr) { continue; }
        ++n_actions_no_nullptr;
      }
      for (auto const& a : ssp_.applicableActions(s)) {
        ++n_applicable_actions;
      }
    } else {
      n_other_states += 1;
    }
  }

  std::cout << "partial SSP size"
            << "  n_internal_states: " << n_internal_states
            << "  n_fringe_and_goal_states: " << n_fringe_and_goal_states
            << "  n_other_states: " << n_other_states
            << "  n_actions: " << n_actions
            << "  n_actions_no_nullptr: " << n_actions_no_nullptr
            << "  n_applicable_actions: " << n_applicable_actions
            << std::endl;
}

void PartialSSPExtended::sparsityStatistics() const {

  auto extract_lifted_action = [](action_t const* a_ptr) {
    std::deque<std::string> action_tokens;
    std::string lifted_name = a_ptr->name();
    boost::erase_all(lifted_name, "(");
    boost::erase_all(lifted_name, ")");
    boost::split(action_tokens, lifted_name, boost::is_any_of(" "));
    lifted_name = action_tokens[0];
    return lifted_name;
  };

  // Variables for statistics
  size_t n_partial_actions_in_envelope = 0;
  size_t n_total_actions_in_envelope = 0;
  std::map<std::tuple<size_t, size_t>, size_t> state_count;
  std::map<std::tuple<size_t, size_t, std::string>, size_t> per_action_state_count;

  for (auto const& [s, s_partial_actions] : partial_space_) {
    if (ssp_.isGoal(s) or not isInternal(s)) {
      continue;
    }

    // Count up the number of partial/total actions applicable in s, and the number of
    // partial/total actions partitioned by the lifted action name
    size_t n_partial_actions = 0;
    size_t n_total_actions = 0;
    std::unordered_map<std::string, size_t> n_partial_actions_per_lifted_action;
    std::unordered_map<std::string, size_t> n_total_actions_per_lifted_action;
    for (auto const* a_ptr : s_partial_actions) {
      if (a_ptr == nullptr) { continue; }
      ++n_partial_actions;
      ++n_partial_actions_per_lifted_action[extract_lifted_action(a_ptr)];
    }
    for (auto const& a : ssp_.applicableActions(s)) {
      ++n_total_actions;
      ++n_total_actions_per_lifted_action[extract_lifted_action(&a)];
    }

    // Accumulate the statistics
    n_partial_actions_in_envelope += n_partial_actions;
    n_total_actions_in_envelope += n_total_actions;
    ++state_count[std::make_tuple(n_partial_actions, n_total_actions)];
    for (auto const& [action_name, n_total_actions_per_lifted_action_count] : n_total_actions_per_lifted_action) {
      ++per_action_state_count[std::make_tuple(n_partial_actions_per_lifted_action[action_name], n_total_actions_per_lifted_action_count, action_name)];
    }

  }

  // Print out statistics
  std::cout << "<envelope_sparsity_info>\npartial_actions,total_actions\n"
            << n_partial_actions_in_envelope << ","
            << n_total_actions_in_envelope << "\n"
            << "</envelope_sparsity_info>\n";

  std::cout << "<state_sparsity_info>\npartial_actions,total_actions,count\n";
  for (auto const& [stats, count] : state_count) {
    auto const& [partial_actions, total_actions] = stats;
    std::cout << partial_actions << "," << total_actions << "," << count << "\n";
  }
  std::cout << "</state_sparsity_info>\n";

  std::cout << "<per_action_state_sparsity_info>\npartial_actions,total_actions,lifted_action,count\n";
  for (auto const& [stats, count] : per_action_state_count) {
    auto const& [partial_actions, total_actions, action_name] = stats;
    std::cout << partial_actions << "," << total_actions << "," << action_name << "," << count << "\n";
  }
  std::cout << "</per_action_state_sparsity_info>\n";
}


void PartialSSPExtended::statistics() const {
#if 0
  // Computing these quartiles can require some seconds so skipping it for the real experiments
  // If they are needed, then the time keeping should be stopped
  size_t total_non_fringe_transitions = 0;
  size_t used_non_fringe_transitions = 0;
  std::vector<float> obeserved_perc_used;

  for (auto const& pair : partial_space_) {
    state_t const& s = pair.first;
    auto const& considered_actions = pair.second;
    size_t num_applicable_actions = 0;

    // TODO: SSP might have already been deallocated at this point. GCC compiled code will
    // terminate with:
    //    pure virtual method called
    //    terminate called without an active exception
    for ([[maybe_unused]] action_t const& a : ssp_.applicableActions(s)) {
      num_applicable_actions++;
    }

    if (!isFringe(s)) {
      total_non_fringe_transitions += num_applicable_actions;
      used_non_fringe_transitions += considered_actions.size();
      float used_perc = 100 * considered_actions.size() / (float) num_applicable_actions;
      obeserved_perc_used.push_back(used_perc);
    }
  }

  if (obeserved_perc_used.size() == 0) { return; }

  // Computing quartiles by sorting and then getting the n-th element
  std::sort(obeserved_perc_used.begin(), obeserved_perc_used.end());

  float q25 = obeserved_perc_used[obeserved_perc_used.size()/4];
  float q50 = obeserved_perc_used[obeserved_perc_used.size()/2];
  float q75 = obeserved_perc_used[3 * obeserved_perc_used.size()/4];
  float total_ratio_used = used_non_fringe_transitions / (float) total_non_fringe_transitions;

  std::cout << "[PartialSSPExtended::stats] total non-fringe transitions = " << used_non_fringe_transitions
            << " (out of " << total_non_fringe_transitions << ") "
            << "-- saved " << (100 * (1 - total_ratio_used))  << "%\n"
            << "[PartialSSPExtended::stats] non-fringe transitions used " << "q25 = " << q25 << "%\n"
            << "[PartialSSPExtended::stats] non-fringe transitions used " << "q50 = " << q50 << "%\n"
            << "[PartialSSPExtended::stats] non-fringe transitions used " << "q75 = " << q75 << "%\n";
#endif
}


void PartialSSPExtended::fancyPolicyDebugRec(state_t const& s, HashsetState& open_or_closed, bool partial_pi,
    hash_t const* v, std::string indentation) const
{
  if (gpt::problem->isGoal(s)) {
    std::cout << indentation << "GOAL: " << s.toStringFull(gpt::problem)
              << std::endl;
    open_or_closed.insert(s);
    return;
  }

  if (open_or_closed.find(s) != open_or_closed.end()) {
    std::cout << indentation << s.toStringFull(gpt::problem)
              << "  already printed" << std::endl;
    return;
  }

  open_or_closed.insert(s);

  assert(partial_pi || cur_policy_.find(s) != cur_policy_.end());
  if (cur_policy_.find(s) == cur_policy_.end()) {
    std::cout << indentation << s.toStringFull(gpt::problem) << " -- undefined (?)\n";
    return;
  }

  std::cout << indentation << s.toStringFull(gpt::problem) << ":\n";
  action_t const* pi_s = cur_policy_.find(s)->second;

  if (!pi_s || pi_s->name() == std::string("fringe/d-e")) {
    std::cout << "nullptr\n";
    return;
  }

  // std::cout << a->name();
  if (v != nullptr && pi_s != nullptr) {
    double q_s_pi_s = Bellman::constQValue(s, *pi_s, *v, ssp_);
    double min_q_value = Bellman::constMinQValue(s, *v, ssp_);
    // Being lazy and putting msgs here to be sorted so that they are printed in alphabetical order
    // to make comparison easier
    std::set<std::string> q_val_msgs;

    auto const it = partial_space_.find(s);
    assert(it != partial_space_.end());
    for (action_t const* a : it->second) {
      std::ostringstream ost;

      assert(a != nullptr); // it will fail... adapt code for it
      double q_s_a = Bellman::constQValue(s, *a, *v, ssp_);
      ost << indentation << "  -- " << a->name() << " Q(s,a) = " << q_s_a;
      assert(pi_s != nullptr);
      assert(a != nullptr);
      if (a->name() == pi_s->name()) {
        ost << " == PI(s)";
        assert(min_q_value <= q_s_pi_s);
        if (min_q_value == q_s_pi_s) {
          ost << " all good";
        }
        else if (q_s_pi_s - min_q_value <= gpt::epsilon) {
          ost << " within epsilon";
        }
        else {
          assert(fabs(q_s_pi_s - min_q_value) > gpt::epsilon);
          ost << " pi is NOT GREEDY!";
        }
      }
      if (fabs(q_s_a - q_s_pi_s) <= gpt::epsilon) {
        ost << " epsilon-TIED with pi(s)";
      }
      q_val_msgs.insert(ost.str());
    }
    for (auto const& str : q_val_msgs) {
      std::cout << str << std::endl;
    }
  }
  ProbDistStateHash pr; // CANNOT BE STATIC BECAUSE OF RECURSION
  gpt::problem->expand(*pi_s, s, pr);
  for (auto const& ip : pr) {
    fancyPolicyDebugRec(ip.event(), open_or_closed, partial_pi, v, indentation + "  ");
  }
}


void PartialSSPExtended::fancyPartialPolicyDebug(state_t const& s0, hash_t const* v) const {
  HashsetState open_or_closed_states;
  std::cout << "\nPARTIAL POLICY DUMP:\n";
  fancyPolicyDebugRec(s0, open_or_closed_states, true, v, "");
  std::cout << "DUMP FINISHED\n";
}


/*******************************************************************************
 *
 * planner CG-ILAO
 *
 ******************************************************************************/

PlannerCGiLAOExtended::PlannerCGiLAOExtended(SSPIface const& ssp, heuristic_t& heur, double epsilon,
  std::string const& k, std::string const& expansion_type, std::string const& improvement_type,
  std::string const& n_violation_fix_passes, std::string const& max_col_age,
  std::string const& action_elim)
  : OptimalPlanner(), ssp_(ssp), v_(gpt::initial_hash_size, heur),
    epsilon_(epsilon), partial_ssp_(ssp), solved_from_s0_(false), use_mlo_(false), n_best_goals_(0)
{
  using boost::istarts_with;

  if (k == "inf") {
    k_ = 42; // arbritary
    use_infty_k_ = true;
  }
  else {
    k_ = std::stoi(k);
    use_infty_k_ = false;
  }

  if (k_ == 0) {
    std::cerr << "\n\nError! k must be > 0 or 'inf'\n\n";
    exit(-1);
  }

  std::cout << " [Ctor] PlannerCGiLAOExtended: k_ = " << k_ << std::endl;
  std::cout << " [Ctor] PlannerCGiLAOExtended: use_infty_k_ = " << use_infty_k_ << std::endl;

  if (expansion_type == "bellman") {
    expansion_ = CGiLAOExtendedExpansionType::bellman;
  }
  else if (expansion_type == "bellman-tied") {
    expansion_ = CGiLAOExtendedExpansionType::bellman_tied;
  }
  else if (expansion_type == "complete") {
    expansion_ = CGiLAOExtendedExpansionType::complete;
  }
  else if (istarts_with(expansion_type, "bellman-n-best")) {
    std::deque<std::string> tokens;
    boost::split(tokens, expansion_type, boost::is_any_of("@"));
    assert(tokens[0] == "bellman-n-best");
    expansion_ = CGiLAOExtendedExpansionType::bellman_n_best;
    expansion_n_ = std::stoi(tokens[1]);
    std::cout << "-- bellman-n-best; n = " << expansion_n_ << std::endl;
  }
  else if (istarts_with(expansion_type, "bellman-x-best")) {
    std::deque<std::string> tokens;
    boost::split(tokens, expansion_type, boost::is_any_of("@"));
    assert(tokens[0] == "bellman-x-best");
    expansion_ = CGiLAOExtendedExpansionType::bellman_x_best;
    expansion_x_ = std::stof(tokens[1]);
    std::cout << "-- bellman-x-best; x = " << expansion_x_ << std::endl;
  }
  else if (istarts_with(expansion_type, "bellman-x2-best")) {
    std::deque<std::string> tokens;
    boost::split(tokens, expansion_type, boost::is_any_of("@"));
    assert(tokens[0] == "bellman-x2-best");
    expansion_ = CGiLAOExtendedExpansionType::bellman_x2_best;
    expansion_x_ = std::stof(tokens[1]);
    std::cout << "-- bellman-x2-best; x2 = " << expansion_x_ << std::endl;
  }
  else if (istarts_with(expansion_type, "bellman-x3-best")) {
    std::deque<std::string> tokens;
    boost::split(tokens, expansion_type, boost::is_any_of("@"));
    assert(tokens[0] == "bellman-x3-best");
    expansion_ = CGiLAOExtendedExpansionType::bellman_x3_best;
    expansion_x_ = std::stof(tokens[1]);
    std::cout << "-- bellman-x3-best; x3 = " << expansion_x_ << std::endl;
  }
  else if (istarts_with(expansion_type, "ff")) {
    std::deque<std::string> tokens;
    boost::split(tokens, expansion_type, boost::is_any_of("-"));
    assert(tokens[0] == "ff");
    tokens.pop_front();
    expansion_ = CGiLAOExtendedExpansionType::ff;
    for (auto const& tok : tokens) {
      if (tok == "mlo") {
        use_mlo_ = true;
      } else if (tok == "bg") {
        // TODO should pass number
        n_best_goals_ = 100;
      } else {
        std::cerr << "\n\nCG-iLAO: FF setting '" << tok << "' not recognized\n\n";
        exit(-1);
      }
    }
    std::cout << "-- FF MLO: " << use_mlo_ << std::endl;
    std::cout << "-- FF best_goals: " << n_best_goals_ << std::endl;
  }
  else if (istarts_with(expansion_type, "trial")) {
    std::deque<std::string> tokens;
    boost::split(tokens, expansion_type, boost::is_any_of("-"));
    assert(tokens[0] == "trial");
    tokens.pop_front();
    expansion_ = CGiLAOExtendedExpansionType::trial;
    max_trial_steps_ = std::stoi(tokens[0]);
    std::cout << "-- trial max_trial_steps_: " << max_trial_steps_ << std::endl;
  }
  else {
    std::cerr << "\n\nCG-iLAO: expansion_type '" << expansion_type << "' not recognized\n\n";
    exit(-1);
  }

  // Set up expansion settings on partial_ssp_
  //
  // this is a bit HACKY
  partial_ssp_.setExpansionSettings(expansion_, expansion_n_, expansion_x_);

  // Set up function used for expanding policy
  if (isExpansionFromBellmanFamily()) {
    if (k_ == 1 and !use_infty_k_) {
      expand_policy_func_ = [&](ListOfStates& postorder_traversal) { expandPolicyBellmanK1(postorder_traversal); };
    } else {
      expand_policy_func_ = [&](ListOfStates& postorder_traversal) { expandPolicyBellmanKMoreThan1(postorder_traversal); };
    }
  } else if (expansion_ == CGiLAOExtendedExpansionType::ff) {
    expand_policy_func_ = [&](ListOfStates& postorder_traversal) { expandPolicyFF(postorder_traversal); };
  } else if (expansion_ == CGiLAOExtendedExpansionType::trial) {
    expand_policy_func_ = [&](ListOfStates& postorder_traversal) { expandPolicyTrial(postorder_traversal); };
  } else {
    exit(-1);
  }

  if (improvement_type == "postorder") {
    improvement_ = ImprovementType::postorder;
  }
  else if (improvement_type == "vi") {
    improvement_ = ImprovementType::vi;
  }
  else {
    std::cerr << "\n\nCG-iLAO: improvement_type '" << improvement_type << "' not recognized\n\n";
    exit(-1);
  }

  if (n_violation_fix_passes == "inf") {
    n_violation_fix_passes_ = 1;
    fix_constrs_gap_ = 1;
    use_infty_n_violation_fix_passes_ = true;
    use_infty_fix_constrs_gap_ = false;
  }
  else if (n_violation_fix_passes == "-inf") {
    n_violation_fix_passes_ = 1;
    fix_constrs_gap_ = 1;
    use_infty_n_violation_fix_passes_ = false;
    use_infty_fix_constrs_gap_ = true;
  }
  else if (istarts_with(n_violation_fix_passes, "-")) {
    const auto x = boost::algorithm::erase_all_copy(n_violation_fix_passes, "-");
    std::cout << "=== x = " << x << std::endl;
    n_violation_fix_passes_ = 1;
    fix_constrs_gap_ = std::stoi(x);
    use_infty_n_violation_fix_passes_ = false;
    use_infty_fix_constrs_gap_ = false;
  }
  else {
    n_violation_fix_passes_ = std::stoi(n_violation_fix_passes);
    fix_constrs_gap_ = 1;
    use_infty_n_violation_fix_passes_ = false;
    use_infty_fix_constrs_gap_ = false;
  }
  std::cout << "j = "  << n_violation_fix_passes_ << "; j' = " << fix_constrs_gap_ << std::endl;

  if (n_violation_fix_passes_ == 0) {
    std::cerr << "\n\nError! bad input for n_violation_fix_passes\n\n";
    exit(-1);
  }

  max_col_age_ = std::stoi(max_col_age);

  partial_ssp_.setActionEliminationEnabled(bool(std::stoi(action_elim)));

  if (max_col_age_ > -1 and bool(std::stoi(action_elim))) {
    EXIT("NOT IMPLEMENTED: do EITHER action elim OR stale action removal NOT BOTH");
  }
}

/*
 * Expensive debugging method to make sure that policy_fringe_[fringe_idx_] is a superset of the
 * current policy fringes
 */
bool PlannerCGiLAOExtended::debugIsSetSupersetOfFringe(SetOfStates const& set) {
  SetOfStates debug_fringe;
  ListOfStates debug_traversal;
  partial_ssp_.dfsSimilationOfCurPolicy(ssp_.s0(), v_, debug_fringe, debug_traversal, false, expansion_);
  size_t missing = 0;
  for (state_t const& s : debug_fringe) {
    if (set.find(s) == set.end()) {
      std::cout << "set is missing states s = " << s.toStringFull(gpt::problem) << std::endl;
      missing++;
    }
  }
  return missing == 0;
}

void PlannerCGiLAOExtended::expandPolicyInnerLoop(size_t n_expansions, ExpandFringeFunction& expand_func) {
  size_t expansions_performed = 0;
  for (; policy_fringe_[fringe_idx_].size() > 0 \
         && (expansions_performed < n_expansions || use_infty_k_); ++expansions_performed)
  {
    bool next_idx = !fringe_idx_;
    policy_fringe_[next_idx].clear();

    ListOfStates dummy_postorder_traversal; // TODO(jsch): would be good if dfsSimilation... doesn't need this at all

    SetOfStates non_fringe_states_reached;
    for (auto const& s: policy_fringe_[fringe_idx_]) {
      // We don't track the fringe of the policy precisely, instead we keep a superset of those
      // states to save computation. So we need to check if this is infact a fringe.
      if (!partial_ssp_.isFringe(s)) { continue; }

      expand_func(s, v_, policy_fringe_[next_idx], non_fringe_states_reached);
    }

    // Doing a non-expanding DFS for each non-fringe state reachable in the bellman expansion to
    // find the true fringes -- this needs to happen because an expansion (Bellman or FF) may
    // encounter as one of its successors a non-fringe state, and we need to follow from the
    // non-fringe state to make sure we have a true superset of fringes
    // for (state_t const& s : non_fringe_states_reached) {
    {
      // We keep the same set of open_or_closed states for each DFS call
      // --> if envelopes from non_fringe_states_reached overlap, we avoid going over them in
      //     duplicate this way
      static SetOfStates open_or_closed;
      open_or_closed.clear();
      for (state_t const& s_prime : non_fringe_states_reached) {
        open_or_closed.insert(s_prime);
      }
      // We also keep the same set of reachable fringes so we only have to do one merge at the end
      SetOfStates reachable_fringes_from_non_fringes = {};
      // Do DFS
      for (state_t const& s : non_fringe_states_reached) {
        dummy_postorder_traversal.clear();
        partial_ssp_.dfsSimilationOfCurPolicyRec(s,
                                                 v_,
                                                 reachable_fringes_from_non_fringes,
                                                 0,
                                                 open_or_closed,
                                                 dummy_postorder_traversal,
                                                 false,
                                                 expansion_);
      }
      // Add newly discovered fringes
      policy_fringe_[next_idx].merge(reachable_fringes_from_non_fringes);
    }
    // OLD VERSION: if the reachable states from states in reachable_fringes_from_s overlap
    // this approach will explore those overlaps in duplicate
    //
    //   SetOfStates reachable_fringes_from_s;
    //   partial_ssp_.dfsSimilationOfCurPolicy(s, v_, reachable_fringes_from_s, dummy_postorder_traversal, false, expansion_);
    //   policy_fringe_[next_idx].merge(reachable_fringes_from_s);
    // }

    fringe_idx_ = next_idx;

    assert(debugIsSetSupersetOfFringe(policy_fringe_[fringe_idx_]));
  }

  // std::cout << "[expandPolicyInnerLoop] # expansions = " << expansions_performed << std::endl;
}

/*
 * Do a single DFS traversal of policy and expand the encountered fringes
 *
 * NOTE: if the DFS traversal expands fringe and encounters non-fringe state, it will keep
 * traversing from that state, to ensure postorder_traversal includes all the fringes
 */
void PlannerCGiLAOExtended::expandPolicyBellmanK1(ListOfStates& postorder_traversal) {
  partial_ssp_.dfsSimilationOfCurPolicy(ssp_.s0(), v_, policy_fringe_[fringe_idx_], postorder_traversal, true, expansion_);
}

/*
 * 1. Do DFS traversal of policy and expand the encountered fringes
 * 2. Do k-2 iterations of expanding fringes without full DFS
 * 3. Do final DFS traversal with fringe expansion to get valid postorder_traversal
 *
 * NOTE: since (1.) and (3.) do fringe expansions, we only need to do k-2 expansions in inner loop
 *
 * NOTE: (2.) does not do full DFS, but if it encounters non-fringe state it does a DFS from there
 */
void PlannerCGiLAOExtended::expandPolicyBellmanKMoreThan1(ListOfStates& postorder_traversal) {
  // (1.)
  ListOfStates dummy_postorder_traversal; // TODO(jsch): would be good if dfsSimilation... doesn't need this at all
  partial_ssp_.dfsSimilationOfCurPolicy(ssp_.s0(), v_, policy_fringe_[fringe_idx_], dummy_postorder_traversal, true, expansion_);

  // (2.)
  ExpandFringeFunction f = [&](state_t const& s, hash_t& v, SetOfStates& new_fringes, SetOfStates& reached_non_fringe){
    partial_ssp_.expandFringeWithBellman(s, v, expansion_, new_fringes, &reached_non_fringe);
  };
  expandPolicyInnerLoop(k_-2, f);

  // (3.)
  partial_ssp_.dfsSimilationOfCurPolicy(ssp_.s0(), v_, policy_fringe_[fringe_idx_], postorder_traversal, true, expansion_);
}

/*
 * 1. Do DFS traversal of policy (no fringe expansion)
 * 2. Do k iterations of expanding fringes without full DFS
 * 3. Do final DFS traversal (no fringe expansion) to get valid postorder_traversal
 *
 * NOTE: (2.) does not do full DFS, but if it encounters non-fringe state it does a DFS from there
 */
void PlannerCGiLAOExtended::expandPolicyFF(ListOfStates& postorder_traversal) {
  // (1.)
  ListOfStates dummy_postorder_traversal; // TODO(jsch): would be good if dfsSimilation... doesn't need this at all
  partial_ssp_.dfsSimilationOfCurPolicy(ssp_.s0(), v_, policy_fringe_[fringe_idx_], dummy_postorder_traversal, false, expansion_);

  // (2.)
  std::vector<state_t> ff_best_goals;
  if (expansion_ == CGiLAOExtendedExpansionType::ff) {
    ff_best_goals = partial_ssp_.getBestGoals(v_, n_best_goals_);
  }
  ExpandFringeFunction f = [&](state_t const& s, hash_t& v, SetOfStates& new_fringes, SetOfStates& reached_non_fringe){
    partial_ssp_.expandFringesWithFF(s, v, new_fringes, use_mlo_, ff_best_goals, reached_non_fringe);
  };
  expandPolicyInnerLoop(k_, f);

  // (3.)
  partial_ssp_.dfsSimilationOfCurPolicy(ssp_.s0(), v_, policy_fringe_[fringe_idx_], postorder_traversal, false, expansion_);
}

/*
 * 1. Do DFS traversal of policy (no fringe expansion)
 * 2. Do k iterations of expanding fringes without full DFS
 * 3. Do final DFS traversal (no fringe expansion) to get valid postorder_traversal
 *
 * NOTE: (2.) does not do full DFS, but if it encounters non-fringe state it does a DFS from there
 *
 * TODO(jsch): can probably refactor this with expandPolicyFF
 */
void PlannerCGiLAOExtended::expandPolicyTrial(ListOfStates& postorder_traversal) {
  // (1.)
  ListOfStates dummy_postorder_traversal; // TODO(jsch): would be good if dfsSimilation... doesn't need this at all
  partial_ssp_.dfsSimilationOfCurPolicy(ssp_.s0(), v_, policy_fringe_[fringe_idx_], dummy_postorder_traversal, false, expansion_);

  // (2.)
  ExpandFringeFunction f = [&](state_t const& s, hash_t& v, SetOfStates& new_fringes, SetOfStates& reached_non_fringe){
    partial_ssp_.expandFringesWithTrial(s, v, new_fringes, reached_non_fringe, max_trial_steps_);
  };
  expandPolicyInnerLoop(k_, f);

  // (3.)
  partial_ssp_.dfsSimilationOfCurPolicy(ssp_.s0(), v_, policy_fringe_[fringe_idx_], postorder_traversal, false, expansion_);
}

size_t PlannerCGiLAOExtended::expandPolicy(ListOfStates& postorder_traversal) {
  size_t qvalues_before = gpt::total_computed_qvalues;
  uint64_t tic = get_cputime_usec();

  // NOTE: these methods work on policy_fringe_ and fringe_idx_ in-place
  expand_policy_func_(postorder_traversal);

  cputime_["expandPolicy"] += get_cputime_usec() - tic;
  qvalues_computed_["expandPolicy"] += gpt::total_computed_qvalues - qvalues_before;
  /////////// stats_about_k_.insert(expansions_performed); TODO(jsch)

  assert(!use_infty_k_ || policy_fringe_[fringe_idx_].size() == 0);
  assert(postorder_traversal.size() > 0 or
         v_.value(ssp_.s0()) == gpt::dead_end_value.double_value());
  assert(debugIsSetSupersetOfFringe(policy_fringe_[fringe_idx_]));

  return policy_fringe_[fringe_idx_].size();
}


CGiLAOExtendedImprovementStatus PlannerCGiLAOExtended::improvePolicy(ListOfStates& postorder_traversal,
    bool is_policy_open, SetOfConstrs& cols_to_be_checked)
{
  size_t qvalues_before = gpt::total_computed_qvalues;
  uint64_t tic = get_cputime_usec();

  CGiLAOExtendedImprovementStatus rv{false, -1};

  if (improvement_ == ImprovementType::postorder) {
    assert(postorder_traversal.size() > 0 or
           v_.value(ssp_.s0()) == gpt::dead_end_value.double_value());
    CGiLAOExtendedImprovementStatus residual_pi_changed;
    rv.max_residual = 1 + epsilon_;
    while (rv.max_residual > epsilon_) {
      rv.max_residual = 0;
      partial_ssp_.resetActionEliminationResidual();
      for (state_t const& s : postorder_traversal) {
        // postorder_traversal does not include fringes, goals or dead ends
        residual_pi_changed = partial_ssp_.partialBellmanBackup(s, v_, cols_to_be_checked,
                                                                max_col_age_ >= 0);
        rv.stopped_due_policy_change |= residual_pi_changed.stopped_due_policy_change;
        rv.max_residual = std::max(rv.max_residual, residual_pi_changed.max_residual);
      }
      if (rv.stopped_due_policy_change || is_policy_open) {
        break;
      }
    }
  }
  else if (improvement_ == ImprovementType::vi) {
    rv = partial_ssp_.valueIterationPartialSSPExtended(v_, epsilon_, cols_to_be_checked);
  }

  cputime_["improvePolicy"] += get_cputime_usec() - tic;
  qvalues_computed_["improvePolicy"] += gpt::total_computed_qvalues - qvalues_before;

  return rv;
}

CGiLAOExtendedImprovementStatus PlannerCGiLAOExtended::fixViolatedConstraints(SetOfConstrs& cols_to_be_checked) {
  size_t const qvalues_before = gpt::total_computed_qvalues;
  uint64_t const tic = get_cputime_usec();

  bool policy_changed = false;
  double max_residual = 0.0;
  SetOfConstrs new_cols_to_be_checked;
  size_t pass_iter = 0;
  while (cols_to_be_checked.size() > 0 and (pass_iter < n_violation_fix_passes_ or use_infty_n_violation_fix_passes_)) {
    // Reset, these are specific to each loop
    max_residual = 0.0;
    new_cols_to_be_checked.clear();

    for (auto const& col : cols_to_be_checked) {
      if (partial_ssp_.isEliminated(col)) {
        continue;
      }

      auto const& [s, a] = col;
      const double q_s_a = Bellman::qValue(s, *a, v_, ssp_);
      if (q_s_a < v_.value(s) - gpt::epsilon) {
        if (not partial_ssp_.containsColumn(s, *a)) {
          partial_ssp_.expandInternalStates({col});
        }

        const CGiLAOExtendedImprovementStatus update_status = partial_ssp_.updateV(s, a, q_s_a, v_, new_cols_to_be_checked);

        policy_changed |= update_status.stopped_due_policy_change;
        max_residual = std::max(update_status.max_residual, max_residual);

      }
    }

    cols_to_be_checked = new_cols_to_be_checked;
    ++pass_iter;
  }

  cputime_["fixViolatedConstraints"] += get_cputime_usec() - tic;
  qvalues_computed_["fixViolatedConstraints"] += gpt::total_computed_qvalues - qvalues_before;

  assert(!use_infty_n_violation_fix_passes_ or max_residual < epsilon_);  // with infty passes the residual should be brought below epsilon
  return {policy_changed, max_residual};
}

void PlannerCGiLAOExtended::solve() {
  size_t total_iterations = 0;
  // size_t deadline_counter = 0;

  // Helper variable to keep track of interesting measurements
  uint64_t start = get_cputime_usec();
  init_v_s0_ = v_.value(ssp_.s0());

  SetOfConstrs cols_to_be_checked;

  /*
   * TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
   * For ilao expansion. Integrate somewhere else
   */
  ListOfStates postorder_traversal;

  size_t n_open_states = 1;
  CGiLAOExtendedImprovementStatus improvement_status{true, gpt::dead_end_value.double_value()};

  bool termination_condition = false;

  while (not termination_condition) {
    gpt::checkDeadline();
    ++total_iterations;

    // Remove stale columns **if there are no outstanding constraint violations**
    //
    // cols_to_be_checked.empty() ==> (V <= V*)
    //
    // V <= V* is required for column removal, otherwise we lose optimality
    if (cols_to_be_checked.empty()) {
      n_stale_col_removals_ += partial_ssp_.removeStaleColumns(max_col_age_);
    }

    /*******************************************************************************
     * Policy Expansion
     ******************************************************************************/
    D(std::cout << "**** Policy Expansion ****\n");
    n_open_states = expandPolicy(postorder_traversal);

    /*******************************************************************************
     * Policy Improvement
     * - Optimizing the current search envelop
     * - can be until policy changes are detected
     ******************************************************************************/
    D(std::cout << "**** Policy Improvement on the following partial SSP****\n";);
    improvement_status = improvePolicy(postorder_traversal, n_open_states > 0, cols_to_be_checked);

    /*******************************************************************************
     * Fix Violated Constraints from VI LP
     * - may add new actions to the partial ssp
     *
     * CAREFUL: may affect improvement_status
     ******************************************************************************/
    if (const bool expand_and_improve_finished = n_open_states == 0
                                              && !improvement_status.stopped_due_policy_change
                                              && improvement_status.max_residual < epsilon_;
        (expand_and_improve_finished and cols_to_be_checked.size() > 0) or
        (!use_infty_fix_constrs_gap_ and total_iterations % fix_constrs_gap_ == 0)) {
      const CGiLAOExtendedImprovementStatus fix_status = fixViolatedConstraints(cols_to_be_checked);
      improvement_status.max_residual = std::max(improvement_status.max_residual, fix_status.max_residual);
      improvement_status.stopped_due_policy_change |= fix_status.stopped_due_policy_change;
      assert(cols_to_be_checked.empty() || improvement_status.max_residual > epsilon_);
    }

    // Termination condition
    termination_condition = n_open_states == 0 \
                         && !improvement_status.stopped_due_policy_change \
                         && improvement_status.max_residual < epsilon_ \
                         && cols_to_be_checked.empty();

    if (total_iterations % 100 == 0) {
      std::cout << "[cg-ilao::solve]"
                << " ite = " << total_iterations
                << "  n_open_states = " << n_open_states
                << "  pi changed? " << (improvement_status.stopped_due_policy_change ? "Y" : "n")
                << "  max_residual = " << improvement_status.max_residual
                << "  V(s0) after improv = " << v_.value(ssp_.s0())
                << "  cols_to_be_checked.size() = " << cols_to_be_checked.size()
                << "  n_stale_col_removals [DUP] = " << n_stale_col_removals_
                << "  n_cols_added [DUP] = " << partial_ssp_.getNColsAdded() // DUP=DUPLICATE COLS
                << "  n_eliminated_actions = " << partial_ssp_.getNEliminatedActions()
                << std::endl;
    }

  }

  std::cout << "[cg-ilao::solve] DONE.\n";
  solved_from_s0_ = true;

  // Use the line(S) below for debug only!!! It is VERY EXPENSIVE and should only be executed to find
  // potential issues.
  // D(partial_ssp_.dump(&v_); partial_ssp_.dumpPolicy(&v_););

  std::cout << "\nCSVHACK"
            << "," << gpt::__ppddl_domain_filename
            << "," << gpt::__ppddl_problem_filename
            << "," << gpt::seed
            << "," << gpt::algorithm
            << "," << gpt::heuristic
            << "," << v_.value(ssp_.s0())
            << "," << get_cputime_usec()
            << "," << gpt::heur_ptr->totalCalls()
            << "," << gpt::total_computed_qvalues
            << "\n\n";

  std::cout << "max unique qvalues: " << partial_ssp_.getMaxUniqueQvalues() << std::endl;

  partial_ssp_.printActionEliminationInfo();

  uint64_t total_up_to_now = get_cputime_usec() - start;
  std::cout << "[cg-ilao::solve] cputime profile:\n";
  for (auto const& pair : cputime_) {
    std::cout << "[cputime " << pair.first << "] " << pair.second << " -- "
              << (100 * pair.second / (float) total_up_to_now) << "% of total\n";
  }

  std::cout << "[COLUMN REMOVAL INFO]"
            << " n_stale_col_removals [DUP] = " << n_stale_col_removals_
            << " n_cols_added [DUP] = " << partial_ssp_.getNColsAdded()
            << " n_cols_in_part_SSP = " << partial_ssp_.getNColsInPartSSP()
            << " n_actions_eliminated = " << partial_ssp_.getNEliminatedActions() << std::endl;

  dumpPolicyEnvelopeInfo(*this, ssp_);

  partial_ssp_.printPartialSSPSize();
  partial_ssp_.sparsityStatistics();
}
