#include "planner_iface.h"
#include "ilao.h"
#include "policy_envelope_printer.h"

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ext/mgpt/states.h"
#include "../utils/die.h"

#include <queue>

/*******************************************************************************
 *
 * ExplicitGraph
 *
 ******************************************************************************/
ExplicitGraph::ExplicitGraph(SSPIface const& ssp, hash_t& v) : ssp_(ssp), v_(v),
  policy_is_closed_(false), marked_token_(0), v_pr_(ILAO_INITIAL_NUM_PROB_DIST)
{
  flags_[ssp_.s0()].status = StateStatus::NON_TERMINAL_TIP;
}



bool ExplicitGraph::updateGraphBestActions() {
  bool policy_changed = false;
  policy_is_closed_ = true;

  clearMarkedFlags();
  _D(DEBUG_ILAO,
    std::cout << "[ExplicitGraph::updateGraphBestActions] marked_token_ == "
              << marked_token_ << std::endl;)

  state_t const& s0 = ssp_.s0();
  StateFlags& s0_flags = flags_[s0];
  assert(s0_flags.status != StateStatus::UNASSIGNED);

  // Initialize the search using the roots
  ListOfStates open;
  open.push_back(s0);
  s0_flags.marked = marked_token_;

  size_t deadline_counter = 0;
  while (!open.empty()) {
    gpt::incCounterAndCheckDeadlineEvery(deadline_counter, 50);
    state_t cur_s = open.front();
    open.pop_front();

    StateFlags& cur_s_flags = flags_[cur_s];
    assert(cur_s_flags.status != StateStatus::UNASSIGNED);
    assert(cur_s_flags.marked == marked_token_);

    _D(DEBUG_ILAO,
      std::cout
        << "  cur_s = " << cur_s.toStringFull(gpt::problem)
        << " [" << cur_s_flags.toString() << "]"
        << "  --  now |open| = " << open.size()
        << std::endl;
      )

    // TERMINAL STATES: Either goal or dead end
    if (cur_s_flags.status == StateStatus::TERMINAL_TIP
        || cur_s_flags.status == StateStatus::DEAD_END)
    {
      continue;
    }

    if (cur_s_flags.status == StateStatus::NON_TERMINAL_TIP) {
      policy_is_closed_ = false;
      continue;
    }


    // All goals should be marked as TERMINAL_TIP
    assert(!ssp_.isGoal(cur_s));

    assert(cur_s_flags.status == StateStatus::INTERNAL);
    double min_q_value = 0;
    action_t const* a_greedy = nullptr;
    std::tie(a_greedy, min_q_value) = Bellman::greedyActionAndMinQValue(cur_s, v_, ssp_);

    if (cur_s_flags.action != a_greedy) {
      cur_s_flags.action = a_greedy;
      policy_changed = true;
    }

    // NEW DEAD END
    if (a_greedy == nullptr) {
      // This state is a dead end either because there is no applicable action
      // or because it is too expensive
      assert(min_q_value == gpt::dead_end_value.double_value());
      cur_s_flags.status = StateStatus::DEAD_END;
      _D(DEBUG_ILAO,
          std::cout << "[ExplicitGraph::updateGraphBestActions] Marking "
                    << cur_s.toStringFull(gpt::problem)
                    << " as dead end (min Q(s,a) == " << min_q_value
                    << " and cur_s.hasApplicableActions == "
                    << ssp_.hasApplicableActions(cur_s)
                    << std::endl;)
    }
    else {
      assert(min_q_value < gpt::dead_end_value.double_value());
      assert(cur_s_flags.action != nullptr);

      // Finding all the descendants of cur_s that are not marked and adding
      // then to the open list. Note that we're reusing the ProbDist pr_
      // built previously to check if all descendants of (cur_s,
      // cur_s_flags.action) are solved
      ssp_.expand(*cur_s_flags.action, cur_s, pr_);
      for (auto const& ip : pr_) {
        state_t const& s_prime = ip.event();
        StateFlags& s_prime_flags = flags_[s_prime];
        if (s_prime_flags.marked == marked_token_) {
          continue;
        }

        assert(s_prime_flags.status != StateStatus::UNASSIGNED);
        s_prime_flags.marked = marked_token_;
        open.push_back(s_prime);
      }
    }
  }
  return policy_changed;
}


HashsetState ExplicitGraph::policyEnvelop() const {
  HashsetState pi_envelop;
  std::queue<state_t> q;

  q.push(ssp_.s0());
  pi_envelop.insert(ssp_.s0());

  while (!q.empty()) {
    state_t s = q.front();
    q.pop();

    _D(DEBUG_ILAO, std::cout << "Poped " << s.toStringFull(gpt::problem) << ": ";)

    if (ssp_.isGoal(s)) {
      _D(DEBUG_ILAO, std::cout << "GOAL" << std::endl;)
      continue;
    }
    if (!ssp_.hasApplicableActions(s)) {
      _D(DEBUG_ILAO, std::cout << "no actions" << std::endl;)
      continue;
    }

    action_t const* a = actionForState(s);
    if (a == nullptr) {
      _D(DEBUG_ILAO, std::cout << "a is NULL" << std::endl;)
      continue;
    }

    _D(DEBUG_ILAO, std::cout << "a = " << a->name() << std::endl;)

    static ProbDistStateHash pr;
    ssp_.expand(*a, s, pr);

    for (auto const& ip : pr) {
      state_t const& sp = ip.event();
      if (pi_envelop.find(sp) == pi_envelop.end()) {
        q.push(sp);
        pi_envelop.insert(sp);
      }
    }
  }
  return pi_envelop;
}


void ExplicitGraph::postorderDfsRec(state_t const& s, size_t depth,
      ListOfStates& postorder_traversal)
{
  _D(DEBUG_ILAO, std::cout << "[ilao:postorderDfsRec] Depth " << depth << " state = "
                           << s.toStringFull(gpt::problem) << std::endl);

  static size_t deadline_counter = 0;
  gpt::incCounterAndCheckDeadlineEvery(deadline_counter, 500);
  static size_t max_depth = 1;
  if (max_depth == depth) {
    std::cout << "Reached depth " << depth << std::endl;
    max_depth = max_depth << 1;
  }

  // this mark is cleaned in the end of the method and represents that a state
  // is in the union of open and closed nodes of the search
  StateFlags& s_flags = flags_[s];
  s_flags.marked = marked_token_;

  _D(DEBUG_ILAO, std::cout << "  s = " << s.toStringFull(gpt::problem)
                            << " [" << s_flags.toString() << "]" << std::endl)


  if (s_flags.status == StateStatus::INTERNAL) {
    assert(s_flags.action != nullptr);
    // There is an action in the explicit graph for the current node
    _D(DEBUG_ILAO, std::cout << "  s has action " << s_flags.action->name()
                             << ". Applying it" << std::endl)

    if (v_pr_.size() == depth) {
      v_pr_.emplace_back();
    }

    // FWT: Be careful! pr_ cannot be reused in the recursion!
    ssp_.expand(*s_flags.action, s, v_pr_[depth]);
    for (auto const& ip : v_pr_[depth]) {
      state_t const& s_prime = ip.event();
      StateFlags& descentant_flags = flags_[s_prime];
      if (descentant_flags.marked != marked_token_) {
        _D(DEBUG_ILAO, std::cout << "  recursing on "
                             << s_prime.toStringFull(gpt::problem) << std::endl)
        postorderDfsRec(s_prime, depth+1, postorder_traversal);
      }
    }
  }
  else if (s_flags.status == StateStatus::NON_TERMINAL_TIP) {
    // This state will be expanded, even if that means that no state is added
    // (either because they were all in the graph or the state is a dead end)
    // therefore it will be upgraded to INTERNAL (or DEAD_END)
    //
    // NOTICE: ALL ACTIONS applicable in s are used for the expansion and not
    // only the greedy action!
    assert(s_flags.action == nullptr);
    assert(!ssp_.isGoal(s));

    // Checking if the state is a dead end (either because V(s) >= dead-end-penalty
    // or A(s) == empty-set)
    action_t const* a_greedy = Bellman::greedyAction(s, v_, ssp_);
    if (a_greedy == nullptr) {
      // This state is a dead end
      assert(!ssp_.hasApplicableActions(s)
              || Bellman::minQValue(s, v_, ssp_) >= gpt::dead_end_value.double_value());
      s_flags.status = StateStatus::DEAD_END;
      _D(DEBUG_ILAO, std::cout << "  Is an UNMARKED dead end. Marking it and continuing"
                      << std::endl;)
    }
    else {
      // This state is now an INTERNAL state, therefore, we need to add the
      // reachable states from ALL ACTIONS APPLICABLE ON IT
      s_flags.status = StateStatus::INTERNAL;
      s_flags.action = a_greedy;

      for (auto const& a : ssp_.applicableActions(s)) {
        ssp_.expand(a, s, pr_);
          for (auto const& ip : pr_) {
          // Need to expand graph if necessary
          state_t const& s_prime = ip.event();
          // This creates the flags if necessary
          StateFlags& s_prime_flags = flags_[s_prime];
          if (s_prime_flags.status == StateStatus::UNASSIGNED) {
            _D(DEBUG_ILAO, std::cout << "  Adding new state "
                              << s_prime.toStringFull(gpt::problem)
                              << " as NON_TERMINAL_TIP" << std::endl;)
            // First time the state is seen, so adding it as either a non-terminal
            // tip or as a goal (terminal tip)
            s_prime_flags.status = (ssp_.isGoal(s_prime) ? StateStatus::TERMINAL_TIP
                                                         : StateStatus::NON_TERMINAL_TIP);
            // Marking as visited so there is no chance of seeing this state again
            // during search and deciding to expand it.
            s_prime_flags.marked = marked_token_;
          }
          else {
            // s_prime is a state we already have in the graph, so we don't need
            // to add it. MOREOVER, it won't be marked as visited because we might
            // find it later on in the search and we will need to recurse on it
          }
        }  // For all s_prime s.t. P(s_prime|s,a) > 0
      }  // for all a in A(s)
    }  // s is NOT a dead end
  }  // s is a NON_TERMINAL_TIP state
  else {
    assert(s_flags.status == StateStatus::TERMINAL_TIP
           || s_flags.status == StateStatus::DEAD_END);
    assert(s_flags.action == nullptr);
  }
  postorder_traversal.push_back(s);
}



/*******************************************************************************
 *
 * planner ILAO
 *
 ******************************************************************************/

PlannerILAO::PlannerILAO(SSPIface const& ssp, heuristic_t& heur, double epsilon)
  : OptimalPlanner(), ssp_(ssp), v_(gpt::initial_hash_size, heur),
    epsilon_(epsilon), explicit_graph_(ssp, v_), solved_from_s0_(false),
    total_iterations_(0)
{ }


void PlannerILAO::solve() {
  ListOfStates visited;
  total_iterations_ = 0;
  size_t deadline_counter = 0;
  bool converged = false;

  while (!converged) {
    ++total_iterations_;

    _D(DEBUG_ILAO, std::cout << "[ilao::solve] Iteration #" << total_iterations_ << std::endl;)

    // Expanding the explicit graph until it represents a closed policy wrt the
    // given state s
    while (!explicit_graph_.isPolicyClosed()) {
      gpt::incCounterAndCheckDeadlineEvery(deadline_counter, 50);
      visited.clear();
      explicit_graph_.postorderDfs(ssp_.s0(), visited);
      applyBellmanUpdate(visited);
      explicit_graph_.updateGraphBestActions();
    }

    _D(DEBUG_ILAO,
        std::cout << "[ilao::solve] Current policy is marked as CLOSED. Starting 2nd loop"
                  << std::endl;)

    assert(debugClosedPolicy());

    while (explicit_graph_.isPolicyClosed()) {

      HashsetState pi_envelop = explicit_graph_.policyEnvelop();
      double residual = 1.0 + epsilon_;
      bool policy_changed = false;

      _D(DEBUG_ILAO, std::cout << "  Found new closed policy (envelop size = "
                               << pi_envelop.size() << ". Evaluating it"
                               << std::endl;)

      while (!policy_changed && residual > epsilon_) {
        assert(debugClosedPolicy());
        gpt::incCounterAndCheckDeadlineEvery(deadline_counter, 50);
        residual = 0.0;
        for (state_t const& cur_s : pi_envelop) {
          // Computing the Bellman residual AND Bellman UPDATE
          double residual_cur_s = Bellman::residual(cur_s, v_, ssp_, true);
          if (residual < residual_cur_s) {
            residual = residual_cur_s;
          }
        }
        _D(DEBUG_ILAO, std::cout << "    Residual = " << residual << std::endl)
        policy_changed = explicit_graph_.updateGraphBestActions();
      }

      if (!policy_changed && residual <= epsilon_) {
        // We found the optimal solution
        assert(explicit_graph_.isPolicyClosed());
        assert(debugClosedPolicy());
        converged = true;
        break;
      }
    }
  }
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

  explicit_graph_.printPartialSSPSize();
  explicit_graph_.sparsityStatistics();
  // explicit_graph_.dumpPartialSSPStateHistogram();
  // explicit_graph_.dumpPartialSSPActionHistogram();
  dumpPolicyEnvelopeInfo(*this, ssp_);

  _D(DEBUG_ILAO, std::cout << "[iLAO::solve] DONE. pi: closed? "
                           << (debugClosedPolicy() ? "yes" : "NO")
                           << std::endl;
  )
  // explicit_graph_.dump();
  // std::cout << "===============\n";
  // explicit_graph_.fancyPartialPolicyDebug(ssp_.s0());
  solved_from_s0_ = true;
}


bool PlannerILAO::debugClosedPolicy() const {
  HashsetState open_close_list;
  HashsetState states_with_no_actions;

  std::queue<state_t> q;

  q.push(ssp_.s0());
  open_close_list.insert(ssp_.s0());

  while (!q.empty()) {
    state_t s = q.front();
    q.pop();

    if (ssp_.isGoal(s)) continue;
    if (!ssp_.hasApplicableActions(s)
        || Bellman::constMinQValue(s, v_, ssp_) >= gpt::dead_end_value.double_value())
    { continue; }

    action_t const* a = explicit_graph_.actionForState(s);
    if (a == nullptr) {
      std::cout << s.toStringFull(gpt::problem)
                << " DOES NOT HAVE AN ACTION and its constMinQValue == "
                << Bellman::constMinQValue(s, v_, ssp_) << std::endl;
      states_with_no_actions.insert(s);
      continue;
    }

    static ProbDistStateHash pr;
    ssp_.expand(*a, s, pr);

    for (auto const& ip : pr) {
      state_t const& sp = ip.event();
      if (open_close_list.find(sp) == open_close_list.end()) {
        q.push(sp);
        open_close_list.insert(sp);
      }
    }
  }

  if (states_with_no_actions.size() > 0) {
    std::cout << "[ilao::debugClosedPolicy] Policy is NOT closed. States without actions:\n";
    for (auto const& it : states_with_no_actions) {
      std::cout << "  " << it.toStringFull(gpt::problem) << std::endl;
    }
    explicit_graph_.dump();
  }
  return states_with_no_actions.size() == 0;
}


void ExplicitGraph::fancyPartialPolicyDebug(state_t const& s0) const {
  HashsetState open_or_closed_states;
  std::cout << "\nPARTIAL POLICY DUMP:\n";
  fancyPolicyDebugRec(s0, open_or_closed_states, true, "");
  std::cout << "DUMP FINISHED\n";
}


void ExplicitGraph::fancyPolicyDebugRec(state_t const& s, HashsetState& open_or_closed,
                      bool partial_pi, std::string indentation) const
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

  action_t const* pi_s = actionForState(s);

  std::cout << indentation << s.toStringFull(gpt::problem) << ":\n";

  if (!pi_s || pi_s->name() == std::string("fringe/d-e")) {
    std::cout << "nullptr\n";
    return;
  }

  // std::cout << pi_s->name();
  double q_s_pi_s = Bellman::constQValue(s, *pi_s, v_, ssp_);
  double min_q_value = Bellman::constMinQValue(s, v_, ssp_);
  // Being lazy and putting msgs here to be sorted so that they are printed in alphabetical order
  // to make comparison easier
  std::set<std::string> q_val_msgs;

  for (action_t const& a : ssp_.applicableActions(s)) {
    std::ostringstream ost;

    double q_s_a = Bellman::constQValue(s, a, v_, ssp_);
    ost << indentation << "  -- " << a.name() << " Q(s,a) = " << q_s_a;
    assert(pi_s != nullptr);
    if (a.name() == pi_s->name()) {
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

  ProbDistStateHash pr; // CANNOT BE STATIC BECAUSE OF RECURSION
  gpt::problem->expand(*pi_s, s, pr);
  for (auto const& ip : pr) {
    fancyPolicyDebugRec(ip.event(), open_or_closed, partial_pi, indentation + "  ");
  }
}
