#include <cassert>
#include <math.h>
#include <set>
#include <stdlib.h>

#include "planner_iface.h"
#include "tvi.h"

#include "../utils/die.h"
#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ext/mgpt/states.h"


/*******************************************************************************
 *
 * planner TVI
 *
 ******************************************************************************/

PlannerTVI::PlannerTVI(SSPIface const& ssp, heuristic_t& heur, double epsilon,
    SccAlgorithm scc_alg)
  : OptimalPlanner(), ssp_(ssp), v_(gpt::initial_hash_size, heur),
    epsilon_(epsilon), scc_alg_(scc_alg), solved_(false)
{ }


/******************************************************************************
 *
 * Kosaraju SCC and helper methods
 */

// static
std::vector<HashsetState>
PlannerTVI::kosarajuStronglyConnectedComponentsInPostOrder(SSPIface const& ssp)
{
  std::cout << "[" << get_human_readable_timestamp()
            << "] Searching for reachable states..." << std::endl;

  /*
   * Reachability Analysis From s0 ordered by exit time, i.e., post-order
   * traversal
   */
  ListOfStates orderedDfsFromS0 = depthFirstSearchPostOrder(ssp, ssp.s0());

  std::cout << "[" << get_human_readable_timestamp()
            << "] Done! Building reverse graph..." << std::endl;

  /*
   * Caching the calls for the reverse graph so that we don't have to search
   * for actions that can add a given state
   */
  typedef std::unordered_map<state_t, HashsetState, hashState>
                                                        HashStateToSetOfStates;
  HashStateToSetOfStates reachesInReverseGraph;
  ProbDistState pr;

  for (auto const& s : orderedDfsFromS0) {
    gpt::checkDeadline();
    if (!ssp.isGoal(s)) {
      for (action_t const& a : ssp.applicableActions(s)) {
        ssp.expand(a, s, pr);
        for (auto const& ip : pr) {
          reachesInReverseGraph[ip.event()].insert(s);
        }  // ip in result of ia applied in s
      }  // for each action a \in A(s)
    }  // s is not a goal
  }  // for each node in the ordered dfs traversal
  std::cout << "[" << get_human_readable_timestamp()
            << "] Done! Starting search in the reverse graph..."
            << std::endl;

  /* Backward reachability analysis using the exit time from the DFS used in
   * the 1st analysis */
  std::stack<HashsetState> stack_sccs;
  size_t size_largest_scc = 0;
  while (!orderedDfsFromS0.empty()) {
    HashsetState scc;
    std::stack<state_t> toExplore;
    toExplore.push(orderedDfsFromS0.front());
    orderedDfsFromS0.pop_front();

//    std::cout << "Building SCC starting from ";
//    toExplore.top()->state()->full_print(std::cout, &problem_, false);
//    std::cout << std::endl;

    gpt::checkDeadline();
    while (!toExplore.empty()) {
      state_t const s = toExplore.top();
      toExplore.pop();
      scc.insert(s);

      if (scc.size() % 100 == 0) {
        gpt::checkDeadline();
      }

//        std::cout << "[" << get_human_readable_timestamp()
//                  << "] SCC size = " << scc.size() << std::endl;

//      std::cout << "\tAdding " << s.toStringFull(gpt::problem_, false)
//                << std::endl;

      ListOfStates::iterator it = orderedDfsFromS0.begin();
      while (it != orderedDfsFromS0.end()) {
        if (reachesInReverseGraph[s].find(*it)
              != reachesInReverseGraph[s].end())
        {
//          std::cout << "\t\tReachable "
//                    << it->toStringFull_print(gpt::problem, false) << std::endl;

          toExplore.push(*it);
          // Erase and move the iterator forward
          ListOfStates::iterator aux = it;
          ++it;
          orderedDfsFromS0.erase(aux);
        }
        else {
//          std::cout << "\t\tNOT Reachable "
//                    << it->toStringFull_print(gpt::problem, false) << std::endl;
          ++it;
        }
      }
    }
    if (size_largest_scc < scc.size()) {
      size_largest_scc = scc.size();
    }
    stack_sccs.push(scc);
  }

  std::vector<HashsetState> ordered_sccs;
  while (!stack_sccs.empty()) {
    ordered_sccs.push_back(stack_sccs.top());
    stack_sccs.pop();
  }
  return ordered_sccs;
}


/*
 TODO(fwt): The depth of recursion of this method can exceed the limits.
   - Need to convert to iterative DFS by stacking search state (see
     iteTarjanStronglyConnectedComponentsInPostOrder)
   - For now, simply turn off the execution stack limit: $ ulimit -s unlimited
 FWT: NOTICE HOWEVER, that this method is only used by Kosaraju's algorithm
 that is clearly way slower than Tarjan's for all the practical cases. Thus,
 there is no rush in adapting this.
*/
// static
void PlannerTVI::depthFirstSearchPostOrderRecursive(SSPIface const& ssp,
    state_t const& s, HashsetState& visited_nodes, ListOfStates& closed_nodes,
    size_t depth)
{
//  static size_t modVIS = 2;

  visited_nodes.insert(s);
  if (visited_nodes.size() % 100 == 0) {
    gpt::checkDeadline();
  }

  static size_t max_depth = 16;
  if (max_depth == depth) {
    std::cout << "Reached depth " << depth << std::endl;
    max_depth = max_depth << 1;
  }
  // FWT: Before the ProbDist approach, the generation of the successor list was
  // deterministic, i.e., we could regenerate it and restart the search where we
  // were before the recursive call (except when using the hash expansion
  // trick). Now the ProbDist is agnostic to the container, so this guarantee is
  // lost. Therefore, we need to keep the ProbDist around and thus use more
  // memory (and getting speed up from not regenerating the successors).
  //
  // TODO(fwt): See if there is a more efficient way of doing this
  //
  // TODO(fwt): Vector is used here because ProbDistStateArray consumes too much
  // memory, so even depths of 256 the method runs out of memory
  static VectorProbDistState v_pr(TVI_INITIAL_NUM_PROB_DIST);
  if (v_pr.size() == depth) {
    v_pr.emplace_back();
  }

//  if (visited_nodes.size() % modVIS == 0) {
//    std::cout << "Visited size == " << modVIS << std::endl;
//    modVIS *= 2;
//  }

  // By definition a goal state has out-degree 0
  if (!ssp.isGoal(s)) {
    for (action_t const& a : ssp.applicableActions(s)) {
      ssp.expand(a, s, v_pr[depth]);
      for (auto const& ip : v_pr[depth]) {
        state_t const& s_prime = ip.event();
        if (visited_nodes.find(s_prime) == visited_nodes.end()) {
            depthFirstSearchPostOrderRecursive(ssp, s_prime, visited_nodes,
                closed_nodes, depth+1);
          }
        }  // ip in result of ia applied in s
    }  // for each action a \in A(s)
  }  // s is not a goal
  closed_nodes.push_front(s);
}


// static
ListOfStates PlannerTVI::depthFirstSearchPostOrder(SSPIface const& ssp,
    state_t const& s)
{
  std::cout << "Calling depthFirstSearchPostOrderRecursive" << std::endl;
  HashsetState visited_nodes;
  ListOfStates reachable_states;
  depthFirstSearchPostOrderRecursive(ssp, s, visited_nodes, reachable_states,0);

  std::cout << "[PlannerTVI::depthFirstSearchPostOrder] Found "
            << reachable_states.size() << " states" << std::endl;
  _D(DEBUG_TVI,
    for (state_t const& s_r : reachable_states) {
      std::cout << "\t" << s_r.toStringFull(gpt::problem, false) << std::endl;
    });
  return reachable_states;
}
/*****************************************************************************/



/******************************************************************************
 *
 * Recursive Tarjan SCC and helper methods
 */
// static
std::vector<HashsetState>
PlannerTVI::recTarjanStronglyConnectedComponentsInPostOrder(SSPIface const& ssp)
{
  size_t index = 0;
  std::stack<state_t> stack;
  std::vector<HashsetState> ordered_sccs;

  HashStateToPairSizeT indexAndLowindex;
  HashsetState isInStack;

  recTarjanStronglyConnectedComponentsInPostOrderRecursion(ssp, ssp.s0(),
      index, stack, indexAndLowindex, isInStack, ordered_sccs, 0);

  return ordered_sccs;
}


// static
void PlannerTVI::recTarjanStronglyConnectedComponentsInPostOrderRecursion(
      SSPIface const& ssp, state_t const& s, size_t& index,
      std::stack<state_t>& stack,  HashStateToPairSizeT& indexAndLowindex,
      HashsetState& isInStack, std::vector<HashsetState>& ordered_sccs,
      size_t depth)
{
  // first is index, second is lowindex
#define INDEX(param) indexAndLowindex[param].first
#define LOWINDEX(param) indexAndLowindex[param].second
  assert(indexAndLowindex.find(s) == indexAndLowindex.end());
  indexAndLowindex[s] = std::make_pair(index, index);
  index++;
  stack.push(s);
  isInStack.insert(s);

//  std::cout << "[Tarjan] s = " << s.toString() << " idx = " << index
//            << "  lowidx = " << index << std::endl;

  static size_t max_depth = 16;
  if (max_depth == depth) {
    std::cout << "Reached depth " << depth << std::endl;
    max_depth = max_depth << 1;
  }
  // FWT: Before the ProbDist approach, the generation of the successor list was
  // deterministic, i.e., we could regenerate it and restart the search where we
  // were before the recursive call (except when using the hash expansion
  // trick). Now the ProbDist is agnostic to the container, so this guarantee is
  // lost. Therefore, we need to keep the ProbDist around and thus use more
  // memory (and getting speed up from not regenerating the successors).
  //
  // TODO(fwt): See if there is a more efficient way of doing this
  //
  // TODO(fwt): Vector is used here because ProbDistStateArray consumes too much
  // memory, so even depths of 256 the method runs out of memory
  static VectorProbDistState v_pr(TVI_INITIAL_NUM_PROB_DIST);
  if (v_pr.size() == depth) {
    v_pr.emplace_back();
  }

  // A goal state has no out-degree 0 by definition, so we don't need to check
  // anything and every goal state should be a size 1 scc.
  if (!ssp.isGoal(s)) {
    for (action_t const& a : ssp.applicableActions(s)) {
      ssp.expand(a, s, v_pr[depth]);
      for (auto const& ip : v_pr[depth]) {
        state_t const& s_prime = ip.event();

        // FWT: Note that I'm not checking for the duplicate edges that the
        // all-outcomes have, i.e., more than one deterministic action from
        // s to s_prime. If and when that happens, s_prime.index will be
        // defined and s_prime will also be in the stack, so the second case
        // of the if will be triggered. As s_prime.low will be the same for
        // both duplicated actions, then nothing will happen and the
        // algorithm will work just fine.

        // index of s_prime is undefined, i.e., not visited before
        if (indexAndLowindex.find(s_prime) == indexAndLowindex.end()) {
          // Successor s_prime has not yet been visited; recurse on it
          recTarjanStronglyConnectedComponentsInPostOrderRecursion(ssp,
              s_prime, index, stack, indexAndLowindex, isInStack, ordered_sccs,
              depth + 1);

          if (LOWINDEX(s_prime) < LOWINDEX(s)) {
            LOWINDEX(s) = LOWINDEX(s_prime);
          }
        }
        // TODO(fwt): This check has to be efficient, i.e., check mark in w
        else if (isInStack.find(s_prime) != isInStack.end()) {
          // Successor s_prime is in the stack, thus it is part of the
          // current SCC
          if (INDEX(s_prime) < LOWINDEX(s)) {
            LOWINDEX(s) = INDEX(s_prime);
          }
        }
      }  // for each successor of (s, ia)
    }  // for each action a \in A(s)
  }  // if s is not a goal

  // If s is a root node, pop the stack and generate an SCC
  if (LOWINDEX(s) == INDEX(s)) {
//    std::cout << "[Tarjan] Found SCC (idx = " << INDEX(s)
//              << "  lowidx = " << LOWINDEX(s)
//              << " |stack| = " << stack.size()
//              << "). States in found SCC are:" << std::endl;
    HashsetState scc;
    state_t s_prime;
    do {
      s_prime = stack.top();
      stack.pop();
      isInStack.erase(s_prime);
//      std::cout << "\t" << s_prime.toString() << std::endl;
      scc.insert(s_prime);
    } while (s_prime != s);
    ordered_sccs.push_back(scc);
  }
}

/*****************************************************************************/


/******************************************************************************
 *
 * Iterative Tarjan SCC
 */
// static
std::vector<HashsetState>
PlannerTVI::iteTarjanStronglyConnectedComponentsInPostOrder(SSPIface const& ssp)
{
  size_t index = 0;
  std::stack<state_t> stack;
  std::vector<HashsetState> ordered_sccs;

  // first is index, second is lowindex
  HashStateToPairSizeT indexAndLowindex;
  // Same as the Recursive Tarjan, i.e.:
  //#define INDEX(s) indexAndLowindex[s].first
  //#define LOWINDEX(s) indexAndLowindex[s].second

  HashsetState isInStack;

  class RecursionState {
   public:
    state_t const& s;                      // current state
    ActionConstIteWrapper a;               // current action
    ActionConstIteWrapper end_of_a_range;  // end of the range applicableActions
    bool returning_from_call;
    ProbDistIface<state_t>::const_iterator ip;
    RecursionState(state_t const& sP, ActionConstRange range,
        bool returning_from_callP)
      : s(sP), a(range.begin()), end_of_a_range(range.end()),
        returning_from_call(returning_from_callP)
    { }
  };

  std::stack<RecursionState> recursion_state;

  // Initial call

//  std::cout << "[Tarjan] s = " << s.toString() << " idx = " << index
//            << "  lowidx = " << index << std::endl;

  static VectorProbDistState v_pr(TVI_INITIAL_NUM_PROB_DIST);
  size_t max_stack_size = 0;
  size_t max_stack_size_msg = 16;
  recursion_state.push(RecursionState(ssp.s0(),
                                      ssp.applicableActions(ssp.s0()), false));

  fake_recurssion_loop:
  while (!recursion_state.empty()) {
    if (max_stack_size < recursion_state.size()) {
      max_stack_size = recursion_state.size();
      if (max_stack_size == max_stack_size_msg) {
        std::cout << "[tvi:iteTarjan] Max stack size reached "
                  << max_stack_size << std::endl;
        max_stack_size_msg = max_stack_size_msg << 1;
      }
    }
    RecursionState r = recursion_state.top();
    recursion_state.pop();
    size_t const depth = recursion_state.size();
    // True if we need to compute v_pr[depth] and false if we need to continue
    // traversing the already computed v_pr[depth]
    bool generate_pr = true;

    if (!r.returning_from_call) {
      // First time seeing this fake recursion object
//      std::cout << "Recursion state: new call (" << r.s.toString()
//                << ", " << r.ia << ") -- |stack| = "
//                << recursion_state.size() << std::endl;
      assert(indexAndLowindex.find(r.s) == indexAndLowindex.end());
      indexAndLowindex[r.s] = std::make_pair(index, index);
      index++;
      stack.push(r.s);
      isInStack.insert(r.s);
      generate_pr = true;
      if (v_pr.size() == depth) {
        // At this point, the recursion_state stack has the number of calls to
        // be finished (i.e., the current call is not counted). Thus, if they
        // have the same size, then each pr in v_pr is already associated with a
        // recursion state and we need a new one.
        v_pr.emplace_back();
      }
    }
    else {
//      std::cout << "Recursion state: returning call (" << r.s.toString()
//          << ", " << r.ia << ", " << r.ip.event().toString()
//          << ") -- |stack| = " << recursion_state.size() << std::endl;
      if (LOWINDEX(r.ip.event()) < LOWINDEX(r.s)) {
        LOWINDEX(r.s) = LOWINDEX(r.ip.event());
      }
      // Since we're returning, we the current recursion state has an iterator
      // that we need to continue processing and its current position was
      // already dealt with, so incrementing it.
      generate_pr = false;
      ++r.ip;
    }

    if (!ssp.isGoal(r.s)) {
      for (; r.a != r.end_of_a_range; ++r.a) {
        action_t const& a = *r.a;
        if (generate_pr) {
          ssp.expand(a, r.s, v_pr[depth]);
          r.ip = v_pr[depth].begin();
        }

        for (; r.ip != v_pr[depth].end(); ++r.ip) {
          state_t const& s_prime = r.ip.event();


          if (indexAndLowindex.find(s_prime) == indexAndLowindex.end()) {
            // index of s_prime is undefined, i.e., not visited before

//              std::cout << "Recursing on " << s_prime.toString() << std::endl;
            // Successor s_prime has not yet been visited; recurse on it
            r.returning_from_call = true;
            recursion_state.push(r);
            // when x.returning_from_call == false, ip is ignored
            RecursionState next(s_prime, ssp.applicableActions(s_prime), false);
            recursion_state.push(next);
            goto fake_recurssion_loop;
          }
          else if (isInStack.find(s_prime) != isInStack.end()) {
            // Successor s_prime is in the stack, thus it is part of the
            // current SCC
            if (INDEX(s_prime) < LOWINDEX(r.s)) {
              LOWINDEX(r.s) = INDEX(s_prime);
            }
          }
        }  // for each successor of (s, ia)
        // We're done with the current ia, so we need to compute the pr for
        // the next ia
        generate_pr = true;
      }  // for each action a \in A(s)
    }  // if r.s is not a goal

    // If s is a root node of the scc, pop the stack and generate an SCC
    if (LOWINDEX(r.s) == INDEX(r.s)) {
//          std::cout << "[Tarjan] Found SCC (idx = " << INDEX(r.s)
//                    << "  lowidx = " << LOWINDEX(r.s)
//                    << " |stack| = " << stack.size()
//                    << "). States in found SCC are:" << std::endl;
      HashsetState scc;
      state_t s_aux;
      do {
        s_aux = stack.top();
        stack.pop();
        isInStack.erase(s_aux);
//        std::cout << "\t" << s_aux.toString() << std::endl;
        scc.insert(s_aux);
      } while (s_aux != r.s);
      ordered_sccs.push_back(scc);
    }
  }  // while recursion_state stack is not empty
  std::cout << "[tvi:iteTarjan] Max stack size = " << max_stack_size
            << "  --  |v_pr| = " << v_pr.size() << std::endl;
  return ordered_sccs;
}

/*****************************************************************************/


/*
 * SCC Debug method
 */
bool PlannerTVI::debugStronglyConnectedComponentsInPostOrder(
      SSPIface const& ssp, std::vector<HashsetState> const& ordered_sccs)
{
  bool rv = true;
  typedef std::unordered_map<state_t, size_t, hashState> HashStateToSizet;
  HashStateToSizet scc_id;

  size_t id = 0;
  for (HashsetState const& scc : ordered_sccs) {
    for (state_t const& s : scc) {
      assert(scc_id.find(s) == scc_id.end());
      scc_id[s] = id;
    }
    id++;
  }

  for (HashsetState const& scc : ordered_sccs) {
    for (state_t const& s : scc) {
      // By definition a goal state has out-degree 0; therefore, even if there
      // are applicable actions, they should be ignored (as we don't want to
      // leave a goal state).
      if (!ssp.isGoal(s)) {
        size_t s_scc_id = scc_id[s];
        for (action_t const& a : ssp.applicableActions(s)) {
          ProbDistState pr;
          ssp.expand(a, s, pr);
          for (auto const& ip : pr) {
            state_t const& s_prime = ip.event();
            size_t s_prime_scc_id = scc_id[s_prime];
            if (s_prime_scc_id > s_scc_id) {
              rv = false;
              std::cout << "Error! Arc from scc " << s_scc_id
                        << " to " << s_prime_scc_id << std::endl
                        << "\t" << s.toString() << " -- "
                        << a.name() << "--> "
                        << s_prime.toString() << std::endl;
            }
          }
        }
      }
    }
  }
  return rv;
}
