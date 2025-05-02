#include <ostream>
#include <queue>

#include <boost/algorithm/string.hpp>

#include "planner_iface.h"
#include "cg-ilao.h"
#include "policy_envelope_printer.h"

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ext/mgpt/states.h"
#include "../utils/die.h"

/*******************************************************************************
 *
 * Partial SSP
 *
 ******************************************************************************/
PartialSSP::PartialSSP(SSPIface const& ssp) : ssp_(ssp)
{
  partial_space_[ssp.s0()] = {};
  // Base case for the regression_partial_space_
  regression_partial_space_[ssp_.s0()] = {};
}

PartialSSP::~PartialSSP() {
  statistics();
}


void PartialSSP::insertColumn(state_t const& s, action_t const* a) {
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

/*
 * Call this with a_ptr = ARGMIN_{a} Q(s, a)
 */
ImprovementStatus PartialSSP::updateV(state_t const& s, action_t const* a_ptr, const double q_s_a, hash_t& v, SetOfConstrs& cols_to_be_checked) {
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
    if (isPartiallyExpanded(s)) {
      for (action_t const& a : ssp_.applicableActions(s)) {
        if (!containsColumn(s, a)) {
          cols_to_be_checked.insert({s, &a});
        }
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


// TODO: this should probably be factored into Bellman::
double greedyActionAndMinQValueWithTies(state_t const& s, hash_t& hash, SSPIface const& ssp,
    std::vector<action_t const*>& tied_greedy_actions, double epsilon = 0.0)
{
  if (ssp.isGoal(s)) {
    tied_greedy_actions = {nullptr};
    return ssp.terminalCost(s).double_value();
  }

  std::multimap<double, action_t const*> qvalue;
  for (auto const& a : ssp.applicableActions(s)) {
    qvalue.insert({Bellman::qValue(s, a, hash, ssp), &a});
  }
  if (qvalue.size() == 0) {
    tied_greedy_actions = {nullptr};
    return gpt::dead_end_value.double_value();
  }

  auto best_q_it = qvalue.begin();
  double min_q_value = best_q_it->first;
  tied_greedy_actions.clear();

  for (auto const& [q, a] : qvalue) {
    if (q > min_q_value + epsilon) { break; }
    tied_greedy_actions.push_back(a);
  }
  return min_q_value;
}


void PartialSSP::expandFringeWithBellman(state_t const& s, hash_t& v,
    ExpansionType const& expansion_type, SetOfStates& new_fringes, SetOfStates* reached_non_fringe)
{
  assert(isFringe(s));

  // TODO(fwt) make sure that goals are never in the fringes... for now just skipping it
  assert(!ssp_.isGoal(s));

  auto it = partial_space_.find(s);
  // Lazy fringe updates: we don't keep a perfect track of the fringe states, instead we keep a
  // super set of it, therefore, it might be the case that the current state is not a fringe anymore
  if (it != partial_space_.end() && it->second.size() > 0) {
    D(std::cout << "    [expandFringeWithBellman] s was not a fringe... s = "
                << s.toStringFull(gpt::problem) << std::endl;);
    return;
  }

  D(std::cout << "    [expandFringeWithBellman] expanding s = "
              << s.toStringFull(gpt::problem) << std::endl;);

  action_t const* greedy_action = nullptr;
  double min_q_value = -1;
  std::vector<action_t const*> columns_from_s_to_add;

  if (expansion_type == ExpansionType::bellman) {
    std::tie(greedy_action, min_q_value) = Bellman::greedyActionAndMinQValue(s, v, ssp_);
    columns_from_s_to_add.push_back(greedy_action);
  }
  else if (expansion_type == ExpansionType::bellman_tied) {
    min_q_value = greedyActionAndMinQValueWithTies(s, v, ssp_, columns_from_s_to_add, gpt::epsilon);
    assert(columns_from_s_to_add.size() > 0);
    greedy_action = columns_from_s_to_add[0];
  }
  else {
    assert(expansion_type == ExpansionType::complete);
    std::tie(greedy_action, min_q_value) = Bellman::greedyActionAndMinQValue(s, v, ssp_);
    for (action_t const& a : ssp_.applicableActions(s)) {
      columns_from_s_to_add.push_back(&a);
    }
  }

  if (greedy_action == nullptr) {
    // This is a dead end state
    D(std::cout << "Marking as dead end bc greedy action is nullptr for s = "
                << s.toStringFull(gpt::problem) << "\n");
    v.update(s, gpt::dead_end_value.double_value());
    updateCurPolicy(s, nullptr);
    assert(partial_space_.find(s) == partial_space_.end() || partial_space_.find(s)->second.size() == 0);
    // Note: by inserting an action (nullptr in this case) for s we are turning s into a non-fringe state
    insertColumn(s, nullptr);
    assert(!isFringe(s));
    return;
  }

  // Since we did not have any action considered for s, the pi(s) must NOT be defined
  assert(cur_policy_.find(s) == cur_policy_.end());
  updateCurPolicy(s, greedy_action);

  assert(it == partial_space_.end() || it->second.size() == 0);
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


void PartialSSP::dfsSimilationOfCurPolicy(state_t const& s, hash_t& v, SetOfStates& new_fringes,
    ListOfStates& postorder_traversal, bool expand_fringe_states,
    ExpansionType const& expansion_type)
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
void PartialSSP::dfsSimilationOfCurPolicyRec(state_t const& s, hash_t& v, SetOfStates& new_fringes,
    size_t depth, SetOfStates& open_or_closed, ListOfStates& postorder_traversal,
    bool expand_fringe_states, ExpansionType const& expansion_type)
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
void PartialSSP::expandInternalStates(SetOfConstrs const& columns_to_be_added) {
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


double PartialSSP::valueIterationPartialSSP(hash_t& v, double epsilon) {
  double max_residual = 1.0 + epsilon;
  // size_t iter = 0;

  // Not using it...
  SetOfConstrs cols_to_be_checked;

  while (max_residual > epsilon) {
    max_residual = 0;
    // iter++;
    for (auto const& pair : partial_space_) {
      state_t const& s = pair.first;
      if (pair.second.size() == 0) {
        // fringe state
        continue;
      }
      ImprovementStatus status = partialBellmanBackup(s, v, cols_to_be_checked);
      max_residual = std::max(max_residual, status.max_residual);
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
  return max_residual;
}


/*
 * TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
 *
 * Incorporate this function to the rest of the code to remove the horrible duplication everywhere
 */
ImprovementStatus PartialSSP::partialBellmanBackup(state_t const& s, hash_t& v,
    SetOfConstrs& cols_to_be_checked)
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


// Greedy policy is allowed to change during evaluation
ImprovementStatus PartialSSP::greedyPolicyEvaluationPartialSSP(hash_t& v, double epsilon,
    bool stop_on_policy_changes, SetOfConstrs* nullptr_or_cols_to_be_checked)
{
  double max_residual = 1.0 + epsilon;
  [[maybe_unused]] size_t iter = 0;

  std::queue<state_t> to_update;
  SetOfStates updated;

  SetOfStates v_increased;
  SetOfStates v_decreased;

  bool policy_changed = false;

  while (max_residual > epsilon) {
    max_residual = 0;
    iter++;

    updated.clear();
    assert(to_update.size() == 0);

    to_update.push(ssp_.s0());

    D(std::cout << "\n\n[greedyPolicyEvaluationPartialSSP] iter = " << iter << "\n");
    while (!to_update.empty()) {
      // TODO: move pop to the bottom and make s a const ref
      state_t const s = to_update.front();
      to_update.pop();

      D(std::cout << "Partial Bellman Backup for " << s.toStringFull(gpt::problem)
                  << " -- V(s) = " << v.value(s) << std::endl);

      auto updated_it = updated.find(s);
      if (updated_it != updated.end()) { continue; }
      updated.insert(updated_it, s);

      auto const it = partial_space_.find(s);
      assert(it != partial_space_.end());

      auto const& available_actions = it->second;

      assert(!ssp_.isGoal(s));
      // if (ssp_.isGoal(s)) {
      //   assert(ssp_.terminalCost(s) == 0);
      //   continue;
      // }

      assert(available_actions.size() > 0);

      /*
       * TODO: make partialBellmanBackup also return the argmin and replace the code below by it
       */
      if (available_actions.size() == 1 && available_actions[0] == nullptr) {
        // This is a state marked as dead end. Nothing else to do here
        assert(v.value(s) == gpt::dead_end_value.double_value());
        assert(cur_policy_.find(s) != cur_policy_.end());
        assert(cur_policy_.find(s)->second == nullptr);
        continue;
      }

      double min_q_value = gpt::dead_end_value.double_value();
      action_t const* argmin_q_value = nullptr;

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

      assert(cur_policy_.find(s) != cur_policy_.end() || argmin_q_value != nullptr);
      if (cur_policy_[s] != argmin_q_value) {
        D(std::cout << "    policy changed for " << s.toStringFull(gpt::problem) << " from "
                  << (cur_policy_[s] ? cur_policy_[s]->name() : "nullptr") << " to "
                  << (argmin_q_value ? argmin_q_value->name() : "nullptr")
                  << "  -- min_q_value = " << min_q_value << "\n");
        updateCurPolicy(s, argmin_q_value);
        policy_changed = true;
      }

      // Sometimes the only option from some states (in or out side the best policy) is to do loop
      // back and make an improper policy so this assert will fail.
      // assert(cur_policy_[s] != nullptr);

      if (argmin_q_value != nullptr) {
        static ProbDistStateHash pr_s_a;
        ssp_.expand(*argmin_q_value, s, pr_s_a);
        for (auto const& ip : pr_s_a) {
          state_t const& s_prime = ip.event();
          if (updated.find(s_prime) != updated.end() || ssp_.isGoal(s_prime) || isFringe(s_prime))
          {
            continue;
          }
          to_update.push(s_prime);
        }
      }

      double signed_residual = v.value(s) - min_q_value;
      if (signed_residual < 0) {
        // V(s) will increase with this update
        assert(min_q_value > v.value(s));
        D(std::cout << "  V(s) increased from " << v.value(s) << " to " << min_q_value << std::endl);
        v_increased.insert(s);
        // std::cout << "V(s) increased for " << s.toStringFull(gpt::problem) << std::endl;
        max_residual = std::max(max_residual, -signed_residual);
      }
      else if (signed_residual > 0) {
        // V(s) will decrease with this update
        assert(min_q_value < v.value(s));
        D(std::cout << "  V(s) DEcreased from " << v.value(s) << " to " << min_q_value << std::endl);
        v_decreased.insert(s);
        // std::cout << "V(s) decrease for " << s.toStringFull(gpt::problem) << std::endl;
        max_residual = std::max(max_residual, signed_residual);
      }
      else {
        continue;
      }
      v.update(s, min_q_value);
    }

    if (stop_on_policy_changes && policy_changed) {
      // std::cout << "[greedy pi] stopping because policy changed\n";
      break;
    }
  }
  if (nullptr_or_cols_to_be_checked != nullptr) {
    updateColumnsToBeChecked(v_increased, v_decreased, *nullptr_or_cols_to_be_checked);
  }
  // std::cout << "-- |cols_to_be_checked| == " << cols_to_be_checked.size() << std::endl;
  return {stop_on_policy_changes && policy_changed, max_residual};
}


void PartialSSP::updateColumnsToBeChecked(SetOfStates const& v_increased, SetOfStates const& v_decreased,
    SetOfConstrs& cols_to_be_checked)
{
  // If V(s) increased, then all its missing columns should be checked because
  // the upper bound on Q(s,a) increased
  for (state_t const& s : v_increased) {
    if (!isPartiallyExpanded(s)) { continue; }
    for (action_t const& a : ssp_.applicableActions(s)) {
      if (!containsColumn(s, a)) {
        cols_to_be_checked.insert({s, &a});
      }
    }
  }
  // If V(s) decreased, then Q(s',a') s.t. P(s|s',a') > 0 might also have
  // decreased (might because the other results of a could have increased)
  // so we need to check all the missing (s',a')
  for (state_t const& s : v_decreased) {
    // Populating the cols_to_be_checked with the regression and at the same time pruning the
    // columns already in partial_space_ (lazy prunning)
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
      cols_to_be_checked.insert(col);
      ++it;
    }
  }
}


void PartialSSP::computeGreedyPolicyFringesRec(state_t s, Policy const& pi,
    SetOfStates& new_fringes, SetOfStates& visited)
{
  // static int depth = 0;

  // std::cout << "\n computeGreedyPolicyFringes [depth = " << depth << "] s = "
  //           << s.toStringFull(gpt::problem) << std::endl;
  visited.insert(s);

  if (ssp_.isGoal(s)) { return; }

  if (!containsState(s)) {
    // std::cout << " computeGreedyPolicyFringes [depth = " << depth << "] s = "
    //           << s.toStringFull(gpt::problem) << " -- s not in PartialSSP" << std::endl;
    assert(containsState(s));
  }

  if (isFringe(s)) {
    new_fringes.insert(s);
    return;
  }

  auto const it = pi.find(s);
  assert(it != pi.end());
  action_t const* a = it->second;

  if (a == nullptr) {
    // pi(s) = nullptr, i.e., the planner decided to not apply any action.
    // This happens when:
    // 1. s is a trivial dead end
    // 2. s is part of a non-trivial dead end (wrt dead-end penalty)
    // 3. **pi is an improper policy**
    //
    // Notice that 3 never happens in the regular dynamic programming/heuristic search algorithms
    // BUT **it can happend in the column generation approach** because we might not have actions
    // break from this loopy policy.
    //
    // As an example run ./solver_ssp -r 1 -R 1 -h roc -p cg-ilao:1 fwt_bw_06.ppddl
    // one of its first policies (3 states) is improper (pick-up-from-table(b4) <-> put-down(b4))
#ifndef NDEBUG
    // FWT: calling find multiple times because all of them will be disabled on experiments (--ndebug)
    auto ndebug_it = partial_space_.find(s);
    assert(ndebug_it != partial_space_.end());
    auto const& ndebug_hat_a_s = ndebug_it->second;
    if (std::find(ndebug_hat_a_s.begin(), ndebug_hat_a_s.end(), nullptr) == ndebug_hat_a_s.end()) {
      std::cout << "-- Current policy is improper based on V. No need to panic, this is possible "
                << "in the column generation approach. See code\n";
      // dump();
      // fancyPartialPolicyDebug(cur_policy_, ssp_.s0());
    }
#endif
    return;
  }
  assert(ssp_.isApplicable(s, *a));

  // std::cout << "   a = " << a->name() << std::endl;

  // CANNOT BE STATIC because of the recursive calls!!!!
  ProbDistStateHash pr_s_a;
  ssp_.expand(*a, s, pr_s_a);
  for (auto const& ip : pr_s_a) {
    state_t const& s_prime = ip.event();
    if (visited.find(s_prime) == visited.end()) {
      // ++depth;
      computeGreedyPolicyFringesRec(s_prime, pi, new_fringes, visited);
      // --depth;
    }
  }

}

SetOfConstrs PartialSSP::negativeReducedCostColumns(hash_t& v, SetOfConstrs const& cols_to_be_checked)
  const
{
  SetOfConstrs neg_rd_cols;
  for (auto const& col : cols_to_be_checked) {
    state_t const& s = col.state;
    action_t const* a = col.action_ptr;

    assert(a != nullptr);

    // TODO(fwt): I don't think this is needed but it might avoid computing an extra q-value
    if (containsColumn(s,*a)) {
      // std::cout << "    " << a.name() << " is already in PartialSSP\n";
      continue;
    }
    double q_s_a = Bellman::qValue(s, *a, v, ssp_);
    D(std::cout << "  [RC] (" << s.toStringFull(gpt::problem) << ", " << a->name()
                << ") q(s,a) = " << q_s_a << " V(s) = " << v.value(s)
                << (v.value(s) > q_s_a ? " NEGATIVE" : " ignored") << "\n");

    // Since we are computing epsilon-consistent solutions, the difference should be at least
    // epsilon otherwise the residual will be greater than epsilon
    if (v.value(s) - q_s_a > gpt::epsilon) {
      neg_rd_cols.insert({s, a});
      // Adding a single column is not benefitial for the overall performance of the algorithm
      // according to small experiments AND the cg-dual results
      // break;
    }
  }

  // std::cout << "[negativeReducedCostColumns] #cols_to_be_checked = "
  //           << cols_to_be_checked.size()
  //           << " (" << neg_rd_cols.size() << " had neg rc -- "
  //           << (100 * neg_rd_cols.size() / (float) cols_to_be_checked.size()) << "%)"
  //           << std::endl;
  return neg_rd_cols;
}


SetOfConstrs PartialSSP::negativeReducedCostColumnsFullSearch(hash_t& v,
    SetOfConstrs const* cols_to_be_checked) const
{
  size_t n_states_checked = 0;
  size_t n_cols_checked = 0;
  size_t n_states_with_neg_col = 0;

  SetOfConstrs neg_rd_cols;
  for (auto const& pair : partial_space_) {
    state_t const& s = pair.first;
    D(std::cout << "  [red-cost] " << s.toStringFull(gpt::problem) << std::endl;)
    // TODO(fwt): make this check efficient!
    if (!isPartiallyExpanded(s)) {
      D(std::cout << "    -> !isPartiallyExpanded" << std::endl;)
      continue;
    }

    n_states_checked++;
    bool has_neg_rd_col = false;
    for (action_t const& a : ssp_.applicableActions(s)) {
      // TODO(fwt): make this check efficient!
      if (containsColumn(s, a)) {
        D(std::cout << "    " << a.name() << " is already in PartialSSP\n";)
        continue;
      }

      n_cols_checked++;
      // Not using the const version because it will populate V(s) with H(s) for s not in V
      // Note that for every s' s.t. P(s'|s,a,) > 0:
      // - if s' in partial_space_, then V(s) will have the best bound so far.
      // - if s' NOT in partial_space_, then s' is an artificial goal, i.e., we use H(s') as V(s')
      //   and this should be the case since no Bellman backup was applied on s'
      double q_s_a = Bellman::qValue(s, a, v, ssp_);
      D(std::cout << "    " << a.name() << " q(s,a) = " << q_s_a
                  << " V(s) = " << v.value(s)
                  << "  Q(s,a) < V(s) ? " << (q_s_a < v.value(s) ? " YES (neg RC)" : "no")
                  << "  Q(s,a) - V(s) = " << (q_s_a - v.value(s))
                  << "\n");
      // Since we are computing epsilon-consistent solutions, the difference should be at least
      // epsilon otherwise the residual will be greater than epsilon
      if (v.value(s) - q_s_a > gpt::epsilon) {
        if (cols_to_be_checked != nullptr) {
          if (cols_to_be_checked->find({s, &a}) == cols_to_be_checked->end()) {
            std::cout << "Column (" << s.toStringFull(gpt::problem) << "," << a.name()
                      << ") V(s) = " << v.value(s) << " > q_s_a = " << q_s_a
                      << " has neg RC but not in cols_to_be_checked:\n";
            for (auto const& c : (*cols_to_be_checked)) {
              std::cout << "  " << c.state.toStringFull(gpt::problem) << "," << c.action_ptr->name()
                        << std::endl;
            }
          }
          assert(cols_to_be_checked->find({s, &a}) != cols_to_be_checked->end());
        }
        neg_rd_cols.insert({s, &a});
        has_neg_rd_col = true;
        // Adding a single column is not benefitial for the overall performance of the algorithm
        // according to small experiments AND the cg-dual results
        // break;
      }
    }
    if (has_neg_rd_col) {
      n_states_with_neg_col++;
    }
  }

  D(std::cout << "[negativeReducedCostColumns] #s = " << n_states_checked
            << " (" << n_states_with_neg_col << " had neg rc col -- "
            << (100 * n_states_with_neg_col / (float) n_states_checked) << "%)"
            << " #(s,a) = " << n_cols_checked
            << " (" << neg_rd_cols.size() << " had neg rc -- "
            << (100 * neg_rd_cols.size() / (float) n_cols_checked) << "%)"
            << " -- #cols_to_be_checked = "
            << (cols_to_be_checked ? cols_to_be_checked->size() : 0)
            << std::endl);
  return neg_rd_cols;
}


void PartialSSP::dumpPartialSSPStateHistogram() const {
  size_t n_fringe_states = 0;
  size_t n_internal_states = 0;
  size_t n_partial_states = 0;
  size_t n_goals = 0;

  // Iterate over non-external states and count what kind of state they are
  for (auto const& [s, A_hat] : partial_space_) {
    // Ignore goals (for now), they're a weird edge case
    if (ssp_.isGoal(s)) {
      ++n_goals;
    }
    else if (isFringe(s)) {
      assert(not isInternal(s));
      ++n_fringe_states;
    } else {
      assert(isInternal(s));
      if (isPartiallyExpanded(s)) {
        ++n_partial_states;
      } else {
        ++n_internal_states;
      }
    }
  }

  std::cout << "CSV_STATE_HISTOGRAM,"
            << n_fringe_states << ","
            << n_internal_states << ","
            << n_partial_states << ","
            << n_goals << std::endl;
}


void PartialSSP::dumpPartialSSPActionHistogram() const {
  std::vector<size_t> histogram;
  for (auto const& [s, A_hat] : partial_space_) {
    const size_t n_actions = A_hat.size();
    if (n_actions+1 > histogram.size()) {
      histogram.resize(n_actions+1, 0);
    }
    ++histogram[n_actions];
  }

  std::cout << "CSV_ACTION_HISTOGRAM";
  for (size_t const& n_entries : histogram) {
    std::cout << "," << n_entries;
  }
  std::cout << std::endl;
}


void PartialSSP::dump(hash_t const* v) const {
  std::cout << "==== PartialSSP::dump() ====\n";
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

void PartialSSP::printPartialSSPSize() const {
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

void PartialSSP::sparsityStatistics() const {

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

void PartialSSP::statistics() const {
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

  std::cout << "[PartialSSP::stats] total non-fringe transitions = " << used_non_fringe_transitions
            << " (out of " << total_non_fringe_transitions << ") "
            << "-- saved " << (100 * (1 - total_ratio_used))  << "%\n"
            << "[PartialSSP::stats] non-fringe transitions used " << "q25 = " << q25 << "%\n"
            << "[PartialSSP::stats] non-fringe transitions used " << "q50 = " << q50 << "%\n"
            << "[PartialSSP::stats] non-fringe transitions used " << "q75 = " << q75 << "%\n";
#endif
}


bool PartialSSP::allNegativeReducedCostsAccountedFor(hash_t& v) const {
  SetOfConstrs neg_rd_cols_double_check = negativeReducedCostColumnsFullSearch(v, nullptr);
  if (neg_rd_cols_double_check.size() > 0) {
    std::cout << "\n\n**** FAILED TO FIND THE FOLLOWING NEGATIVE REDUCED COST COLUMNS ****\n";
    for (auto const& col : neg_rd_cols_double_check) {
      std::cout << "  " << col.state.toStringFull(gpt::problem) << ", "
                << (col.action_ptr ? col.action_ptr->name() : "nullptr")
                << std::endl;
      std::cout << "    is s in hat{S}? ";
      if (containsState(col.state)) {
        auto const it = partial_space_.find(col.state);
        assert(it != partial_space_.end());
        std::cout << "yes --  |hat{A}| = " << it->second.size() << "\n";
      }
      else {
        std::cout << "NO\n";
      }
    }
    dump(&v);
    dumpPolicy(&v);
    return false;
  }
  return true;
}


void PartialSSP::fancyPolicyDebugRec(state_t const& s, HashsetState& open_or_closed, bool partial_pi,
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


void PartialSSP::fancyPartialPolicyDebug(state_t const& s0, hash_t const* v) const {
  HashsetState open_or_closed_states;
  std::cout << "\nPARTIAL POLICY DUMP:\n";
  fancyPolicyDebugRec(s0, open_or_closed_states, true, v, "");
  std::cout << "DUMP FINISHED\n";
}


/*
 * Adapted from iLAO* -- More redundant code :(
 */
bool PartialSSP::isCurPolicyClosed(hash_t const* v) const {
  HashsetState open_or_closed;
  HashsetState states_with_no_actions;

  std::queue<state_t> q;

  q.push(ssp_.s0());
  open_or_closed.insert(ssp_.s0());

  while (!q.empty()) {
    state_t const s = q.front();
    q.pop();

    if (ssp_.isGoal(s)) continue;

    auto const ispace = partial_space_.find(s);
    assert(ispace != partial_space_.end());
    if (ispace->second.size() == 0) {
      std::cout << "partial_ssp_[s] exists BUT |hat{s}| = 0 s = " << s.toStringFull(gpt::problem)
                << std::endl;
    }

    auto const ip = cur_policy_.find(s);

    if (ip == cur_policy_.end()) {
      // Policy undefined for s
      states_with_no_actions.insert(s);
      continue;
    }

    action_t const* a = ip->second;
    if (a == nullptr) {
      // Dead end
      continue;
    }

    static ProbDistStateHash pr;
    ssp_.expand(*a, s, pr);

    for (auto const& ip : pr) {
      state_t const& s_prime = ip.event();
      if (open_or_closed.find(s_prime) != open_or_closed.end()) { continue; }
      q.push(s_prime);
      open_or_closed.insert(s_prime);
    }
  }

  if (states_with_no_actions.size() > 0) {
    std::cout << "[cg-ilao::isCurPolicyClosed] Policy is NOT closed. States without actions:\n";
    for (auto const& it : states_with_no_actions) {
      std::cout << "  " << it.toStringFull(gpt::problem) << std::endl;
    }
    dump();
    dumpPolicy(v);
  }
  return states_with_no_actions.size() == 0;
}


/*******************************************************************************
 *
 * planner CG-ILAO
 *
 ******************************************************************************/

PlannerCGiLAO::PlannerCGiLAO(SSPIface const& ssp, heuristic_t& heur, double epsilon)
  : OptimalPlanner(), ssp_(ssp), v_(gpt::initial_hash_size, heur),
    epsilon_(epsilon), partial_ssp_(ssp), solved_from_s0_(false)
{
  expansion_ = ExpansionType::bellman_tied;
}


size_t PlannerCGiLAO::expandPolicy(ListOfStates& postorder_traversal) {
  size_t qvalues_before = gpt::total_computed_qvalues;
  uint64_t tic = get_cputime_usec();

  SetOfStates policy_fringe;
  state_t const s0 = ssp_.s0();
  partial_ssp_.dfsSimilationOfCurPolicy(s0, v_, policy_fringe, postorder_traversal,
                                        true, expansion_);

  cputime_["expandPolicy"] += get_cputime_usec() - tic;
  qvalues_computed_["expandPolicy"] += gpt::total_computed_qvalues - qvalues_before;

  return policy_fringe.size();
}


ImprovementStatus PlannerCGiLAO::improvePolicy(ListOfStates& postorder_traversal,
    bool is_policy_open, SetOfConstrs& cols_to_be_checked)
{
  size_t qvalues_before = gpt::total_computed_qvalues;
  uint64_t tic = get_cputime_usec();

  ImprovementStatus rv{false, -1};

  ImprovementStatus residual_pi_changed;
  rv.max_residual = 1 + epsilon_;
  while (rv.max_residual > epsilon_) {
    rv.max_residual = 0;
    for (state_t const& s : postorder_traversal) {
      // postorder_traversal does not include fringes, goals or dead ends
      residual_pi_changed = partial_ssp_.partialBellmanBackup(s, v_, cols_to_be_checked);
      rv.stopped_due_policy_change |= residual_pi_changed.stopped_due_policy_change;
      rv.max_residual = std::max(rv.max_residual, residual_pi_changed.max_residual);
    }
    if (rv.stopped_due_policy_change || is_policy_open) {
      break;
    }
  }

  cputime_["improvePolicy"] += get_cputime_usec() - tic;
  qvalues_computed_["improvePolicy"] += gpt::total_computed_qvalues - qvalues_before;

  return rv;
}


ImprovementStatus PlannerCGiLAO::fixViolatedConstraints(SetOfConstrs& cols_to_be_checked)
{
  size_t qvalues_before = gpt::total_computed_qvalues;
  uint64_t tic = get_cputime_usec();

  bool policy_changed = false;
  double max_residual = 0.0;
  SetOfConstrs new_cols_to_be_checked;

  for (auto const& col : cols_to_be_checked) {
    auto const& [s, a] = col;
    const double q_s_a = Bellman::qValue(s, *a, v_, ssp_);
    if (q_s_a < v_.value(s) - gpt::epsilon) {
      if (not partial_ssp_.containsColumn(s, *a)) {
        partial_ssp_.expandInternalStates({col});
      }

      const ImprovementStatus update_status = partial_ssp_.updateV(s, a, q_s_a, v_, new_cols_to_be_checked);

      policy_changed |= update_status.stopped_due_policy_change;
      max_residual = std::max(update_status.max_residual, max_residual);

    }
  }

  cputime_["fixViolatedConstraints"] += get_cputime_usec() - tic;
  qvalues_computed_["fixViolatedConstraints"] += gpt::total_computed_qvalues - qvalues_before;

  cols_to_be_checked = new_cols_to_be_checked;
  return {policy_changed, max_residual};
}

void PlannerCGiLAO::solve() {
  size_t total_iterations = 0;
  // size_t deadline_counter = 0;

  state_t const s0 = ssp_.s0();

  // Helper variable to keep track of interesting measurements
  uint64_t start = get_cputime_usec();

  // Columns that **may** violate a constraint, so they must be checked
  SetOfConstrs cols_to_be_checked;

  /*
   * TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
   * For ilao expansion. Integrate somewhere else
   */
  ListOfStates postorder_traversal;

  size_t n_open_states = 1;
  ImprovementStatus improvement_status{true, gpt::dead_end_value.double_value()};

  bool termination_condition = false;

  while (not termination_condition) {
    gpt::checkDeadline();
    ++total_iterations;

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
     ******************************************************************************/
    const ImprovementStatus fix_status = fixViolatedConstraints(cols_to_be_checked);
    improvement_status.max_residual = std::max(improvement_status.max_residual, fix_status.max_residual);
    improvement_status.stopped_due_policy_change |= fix_status.stopped_due_policy_change;
    assert(cols_to_be_checked.empty() || improvement_status.max_residual > epsilon_);

    termination_condition = n_open_states == 0 \
                            && !improvement_status.stopped_due_policy_change \
                            && improvement_status.max_residual < epsilon_;

    if (total_iterations % 100 == 0) {
      std::cout << "[cg-ilao::solve]"
                << " ite = " << total_iterations
                << "  n_open_states = " << n_open_states
                << "  pi changed? " << (improvement_status.stopped_due_policy_change ? "Y" : "n")
                << "  max_residual = " << improvement_status.max_residual
                << "  V(s0) after improv = " << v_.value(s0)
                << "  cols_to_be_checked.size() = " << cols_to_be_checked.size()
                << std::endl;
    }

  }

  std::cout << "[cg-ilao::solve] DONE.\n";
  solved_from_s0_ = true;

  // Use the line(S) below for debug only!!! It is VERY EXPENSIVE and should only be executed to find
  // potential issues.
  // assert(partial_ssp_.isCurPolicyClosed(&v_));
  // assert(partial_ssp_.allNegativeReducedCostsAccountedFor(v_));
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

  uint64_t total_up_to_now = get_cputime_usec() - start;
  std::cout << "[cg-ilao::solve] cputime profile:\n";
  for (auto const& pair : cputime_) {
    std::cout << "  " << pair.first << ": " << pair.second << " -- "
              << (100 * pair.second / (float) total_up_to_now) << "% of total\n";
  }

  partial_ssp_.printPartialSSPSize();
  partial_ssp_.sparsityStatistics();
  // partial_ssp_.dumpPartialSSPStateHistogram();
  // partial_ssp_.dumpPartialSSPActionHistogram();
  dumpPolicyEnvelopeInfo(*this, ssp_);
}
