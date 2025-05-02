#ifndef PLANNER_ILAO_H
#define PLANNER_ILAO_H

#include <iostream>
#include <list>
#include <unordered_map>

#include "planner_iface.h"

#include "../ext/mgpt/actions.h"
#include "../ssps/bellman.h"
#include "../utils/die.h"
#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ssps/ssp_iface.h"
#include "../ssps/ssp_utils.h"

#include <boost/algorithm/string.hpp>  // for sparsityStatistics()

class heuristic_t;

#ifndef DEBUG_ILAO
#define DEBUG_ILAO 0
#endif

#ifdef ILAO_INITIAL_NUM_PROB_DIST
#pragma message("Using ILAO_INITIAL_NUM_PROB_DIST = " STRINGIFY(ILAO_INITIAL_NUM_PROB_DIST))
#else
#define ILAO_INITIAL_NUM_PROB_DIST 128
#endif

#define TOKEN_TYPE size_t

/*******************************************************************************
 *
 * ExplicitGraph
 *
 * Helper class to represent the explicit graph in iLAO*.
 *
 ******************************************************************************/
class ExplicitGraph {
 public:
  ExplicitGraph(SSPIface const& ssp, hash_t& v);
  ~ExplicitGraph() { }

  bool isPolicyClosed() const { return policy_is_closed_; }

  // Returns the policy envelop of the current best policy, i.e., the policy of
  // recommended actions in the explicit graph
  HashsetState policyEnvelop() const;

  // Update the best policy for the current explicit graph. Returns true if the
  // current policy changed (i.e., a new best action for an existing state was
  // found) and false otherwise. Also, this method updates the flag saying if
  // the current policy if closed or not.
  //
  // NOTICE: the explicit graph is not augmented here! This is done by the
  // postorderDfs method
  bool updateGraphBestActions();

  // Search the explicit graph using depth-first search. Whenever a NON_TERMINAL_TIP
  // state s is found, it expands s.
  //
  // NOTICE: expansion here means adding all the possible successors of s, that
  // is, add all the states s' not already in the graph such that there exists
  // a in A(s) s.t. P(s'|s,a) > 0. In other words, all the action applicable from s
  // will be used (as opposed to only the action that minimizes (over a) Q(s,a))
  void postorderDfs(state_t const& s, ListOfStates& visited) {
    clearMarkedFlags();
    postorderDfsRec(s, 0, visited);
  }


  bool hasEntryFor(state_t const& s) const {
    return flags_.find(s) != flags_.end();
  }


  action_t const* actionForState(state_t const& s) const {
    auto const it = flags_.find(s);
    if (it == flags_.end()) {
      return nullptr;
    }
    return it->second.action;
  }


  bool isInternal(state_t const& s) const {
    if (ssp_.isGoal(s)) { return false; }
    auto const& flag = flags_.at(s);
    return flag.status == StateStatus::INTERNAL or flag.status == StateStatus::DEAD_END;
  }


  bool isFringe(state_t const& s) const {
    if (ssp_.isGoal(s)) { return false; }
    auto const& flag = flags_.at(s);
    return flag.status == StateStatus::NON_TERMINAL_TIP;
  }

  // See cg-ilao and cg-ilao-extended for context
  //
  // NOTE:
  // --> we count the give-up action (nullptr) for n_actions, because iLAO* can always give up
  //     but this is NOT counted in n_actions_no_nullptr nor n_applicable_actions
  void printPartialSSPSize() const {
    size_t n_internal_states = 0;
    size_t n_fringe_and_goal_states = 0;
    size_t n_other_states = 0;
    size_t n_actions = 0;
    size_t n_actions_no_nullptr = 0;
    size_t n_applicable_actions = 0;

    // for (auto const& [s, s_partial_actions] : partial_space_) {
    for (auto const& [s, flag] : flags_) {
      if (ssp_.isGoal(s) or isFringe(s)) {
        n_fringe_and_goal_states += 1;
      } else if (isInternal(s)) {
        n_internal_states += 1;

        for (auto const& a : ssp_.applicableActions(s)) {
          ++n_actions;
          ++n_actions_no_nullptr;
          ++n_applicable_actions;
        }

        // GIVE UP ACTION
        ++n_actions;
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

  // See cg-ilao.h
  //
  // HACK: we're ignoring the partial action info (because ilao always adds all actions)
  //       and then outputting the total actions instead
  void sparsityStatistics() const {

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

    for (auto const& [s, flag] : flags_) {
      if (ssp_.isGoal(s) or not isInternal(s)) {
        continue;
      }

      // Count up the number of partial/total actions applicable in s, and the number of
      // partial/total actions partitioned by the lifted action name
      size_t n_partial_actions = 0;
      size_t n_total_actions = 0;
      std::unordered_map<std::string, size_t> n_partial_actions_per_lifted_action;
      std::unordered_map<std::string, size_t> n_total_actions_per_lifted_action;
      // for (auto const* a_ptr : s_partial_actions) {
      //   if (a_ptr == nullptr) { continue; }
      //   ++n_partial_actions;
      //   ++n_partial_actions_per_lifted_action[extract_lifted_action(a_ptr)];
      // }
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
              // << n_partial_actions_in_envelope << ","
              << n_total_actions_in_envelope << ","
              << n_total_actions_in_envelope << "\n"
              << "</envelope_sparsity_info>\n";

    std::cout << "<state_sparsity_info>\npartial_actions,total_actions,count\n";
    for (auto const& [stats, count] : state_count) {
      auto const& [partial_actions, total_actions] = stats;
      // std::cout << partial_actions << "," << total_actions << "," << count << "\n";
      std::cout << total_actions << "," << total_actions << "," << count << "\n";
    }
    std::cout << "</state_sparsity_info>\n";

    std::cout << "<per_action_state_sparsity_info>\npartial_actions,total_actions,lifted_action,count\n";
    for (auto const& [stats, count] : per_action_state_count) {
      auto const& [partial_actions, total_actions, action_name] = stats;
      // std::cout << partial_actions << "," << total_actions << "," << action_name << "," << count << "\n";
      std::cout << total_actions << "," << total_actions << "," << action_name << "," << count << "\n";
    }
    std::cout << "</per_action_state_sparsity_info>\n";
  }

  void dumpPartialSSPStateHistogram() const {
    size_t n_fringe_states = 0;
    size_t n_internal_states = 0;
    const size_t n_partial_states = 0;  // iLAO* does not allow for partial states
    size_t n_goals = 0;

    for (auto const& [state, flag] : flags_) {
      if (ssp_.isGoal(state)) {
        ++n_goals;
      }
      else if (isInternal(state)) {
        ++n_internal_states;
      }
      else if (isFringe(state)) {
        ++n_fringe_states;
      }
    }

    std::cout << "CSV_STATE_HISTOGRAM,"
              << n_fringe_states << ","
              << n_internal_states << ","
              << n_partial_states << ","
              << n_goals << std::endl;
  }

  void dumpPartialSSPActionHistogram() const {
    std::vector<size_t> histogram;

    for (auto const& [state, flag] : flags_) {
      size_t n_actions = 0;

      if (isInternal(state)) {
        for (action_t const& a : ssp_.applicableActions(state)) {
          std::ignore = a;
          ++n_actions;
        }
      }
      else if (isFringe(state)) {
        n_actions = 0;
      }
      else {
        continue;
      }

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


  // DEBUG method that dumps the full content of flags_
  void dump() const {
    std::cout << std::endl;
    for (const auto& f : flags_) {
      std::cout << "[DUMP] " << f.first.toStringFull(gpt::problem) << " -- "
                << f.second.toString() << std::endl;
    }
    std::cout << std::endl;
  }


  /*
   * Replicated from CG-iLAO that was replicated from somewhere else... WHAT A NIGHTMARE!
   */
  void fancyPolicyDebugRec(state_t const& s,
                             HashsetState& open_or_closed, bool partial_pi,
                             std::string indentation) const;
  void fancyPartialPolicyDebug(state_t const& s0) const;

 private:
  enum class StateStatus {UNASSIGNED,
                          INTERNAL,
                          NON_TERMINAL_TIP,
                          TERMINAL_TIP,
                          DEAD_END};
  friend std::ostream& operator<<(std::ostream&, ExplicitGraph::StateStatus);

  /****************************************************************************
   *
   * StateFlags: class to represent the extra information of each state in the
   * ExplicitGraph
   *
   ***************************************************************************/
  class StateFlags {
   public:
    /*
     * Member PUBLIC variables
     */
    StateStatus status;
    TOKEN_TYPE marked;
    action_t const* action;


    /*
     * CTor / Dtor
     */
    StateFlags() : status(StateStatus::UNASSIGNED), marked(0), action(nullptr) { }
    ~StateFlags() { }

    // Debug method
    std::string toString() const {
      std::ostringstream ost;
      ost << "status = " << status
          << " marked = " << marked
          << " action = " << (action ? action->name() : "NULL");
      return ost.str();
    }
  };


  /*
   * Private Methods
   */
  inline void clearMarkedFlags() {
    marked_token_++;
    if (marked_token_ == 0) restartTokens();
  }

  void restartTokens() {
    // Token roll over. The idea is that this should never happen or happen
    // very rarely. If it happens too often, increase the size of the counter
    // to a larger class of int.
    std::cout << "[restartTokens] Mark rollover. "
              << "Change TOKEN_TYPE if it happens too often\n";
    // Cleaning everyone's token to make sure a match won't happen by mistake.
    for (auto& it : flags_) {
      it.second.marked = 0;
    }
    marked_token_ = 1;
  }

  void postorderDfsRec(state_t const& s, size_t depth,
                       ListOfStates& postorder_traversal);

  /*
   * Typedefs
   */
  #if defined(USE_PHMAP)
  #include "../ext/parallel_hashmap/phmap.h"
  typedef phmap::flat_hash_map<state_t, StateFlags> HashStateToFlags;
  #else
  typedef std::unordered_map<state_t, StateFlags, hashState> HashStateToFlags;
  #endif

  typedef std::vector<state_t> VectorOfStates;

  /*
   * Member variables
   */
  SSPIface const& ssp_;
  hash_t& v_;

  bool policy_is_closed_;
  TOKEN_TYPE marked_token_;
  VectorProbDistState v_pr_;
  ProbDistState pr_;
  HashStateToFlags flags_;
};



/*******************************************************************************
 *
 * planner iLAO*: Improved LAO*
 *
 ******************************************************************************/

class PlannerILAO : public OptimalPlanner
{
 public:
  PlannerILAO(SSPIface const& ssp, heuristic_t& heur, double epsilon);

  ~PlannerILAO() {
    std::cout << "[total q-values] " << gpt::total_computed_qvalues << std::endl;
  }

  /*
   * Planner Interface
   */
  action_t const* decideAction(state_t const& s) override {
    if (!solved_from_s0_) {
      solve();
    }
#ifndef NDEBUG
    static auto pi_envelop = explicit_graph_.policyEnvelop();
    assert(pi_envelop.find(s) != pi_envelop.end());
#endif
    return explicit_graph_.actionForState(s);
  }

  action_t const* decideAction(state_t const& s) const override {
    /*
     * Bellman::constGreedyAction is not used here because we already have a
     * policy for the problem inside the explicit graph.
     */
    _D(DEBUG_ILAO, std::cout << "Next C for s = "
                             << s.toStringFull(gpt::problem) << std::endl)
    // Nothing to be done
    if (ssp_.isGoal(s) || !ssp_.hasApplicableActions(s)) return nullptr;

    if (!explicit_graph_.hasEntryFor(s)) {
      // State was not explored by the search. This should not happen in a
      // regular situation since the explicit graph contains a closed policy
      // with respect to s0
      throw PlannerGaveUpException();
    }
    action_t const* a = explicit_graph_.actionForState(s);
    if (a == nullptr) {
      DIE(!ssp_.isGoal(s) && ssp_.hasApplicableActions(s),
          "Unexpected goal/state without actions", -1);
      // Most likely the planner was interrupted before finding a closed
      // policy. If NULL is returned, then the round evaluation will be tagged
      // as dead end reached, what is not the case.
      throw PlannerGaveUpException();
    }
    _D(DEBUG_ILAO, std::cout << "  Best a = " << a->name() << std::endl);
    return a;
  }

  void trainForUsecs(uint64_t max_time_usec) override {
    if (!runForUsec(max_time_usec, [this]() { solve(); })) {
      std::cout << "[iLAO::trainForUsecs]: training finished before "
                << " convergence." << std::endl;
    }
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
  /*
   * iLAO* methods
   */
  // Main method. It expands the initially empty explicit graph until the
  // "optimal" (epsilon-consistent) closed policy from s0 is found.
  void solve();

  // Applies a Bellman Update to all the states in the given list. Used to
  // update the states visited during the post-order DFS
  void applyBellmanUpdate(ListOfStates& visited) {
    for (auto const& s : visited) { Bellman::update(s, v_, ssp_); }
  }

  // Debug method to check if the policy is really closed. This is done without
  // accessing the internal data of ExplicitGraph to make sure it is working
  // properly.
  //
  // TODO(fwt): move this to utils and add a policy interface to ExplicitGraph
  bool debugClosedPolicy() const;

  /*
   * Member variables
   */
  SSPIface const& ssp_;
  hash_t v_;
  double epsilon_;
  ExplicitGraph explicit_graph_;
  bool solved_from_s0_;
  size_t total_iterations_;
};

inline std::ostream& operator<<(std::ostream& os, ExplicitGraph::StateStatus s) {
  using Status = ExplicitGraph::StateStatus;
  switch (s) {
    case Status::UNASSIGNED       : os << "Unassigned";    break;
    case Status::INTERNAL         : os << "Internal";      break;
    case Status::NON_TERMINAL_TIP : os << "Non-term-tip";  break;
    case Status::TERMINAL_TIP     : os << "Terminal-tip";  break;
    case Status::DEAD_END         : os << "Dead-end";      break;
    default                       : os.setstate(std::ios_base::failbit);
  }
  return os;
}
#endif  // PLANNER_ILAO_H
