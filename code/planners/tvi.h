#ifndef PLANNER_TVI_H
#define PLANNER_TVI_H

#include <deque>
#include <iostream>
#include <list>
#include <stack>
#include <unordered_map>
#include <unordered_set>

#include "planner_iface.h"
#include "vi.h"

#include "../ext/mgpt/actions.h"
#include "../ext/mgpt/hash.h"
#include "../ssps/prob_dist_state.h"
#include "../ssps/ssp_adaptor.h"
#include "../ssps/ssp_iface.h"
#include "../ssps/ssp_utils.h"


#ifndef DEBUG_TVI
#define DEBUG_TVI 0
#endif

#ifndef TVI_INITIAL_NUM_PROB_DIST
#define TVI_INITIAL_NUM_PROB_DIST 1024
#endif

class heuristic_t;


/*******************************************************************************
 *
 * planner TVI
 *
 ******************************************************************************/
class PlannerTVI : public OptimalPlanner {
 public:
  enum class SccAlgorithm {KOSARAJU, REC_TARJAN, ITE_TARJAN};

  PlannerTVI(SSPIface const& ssp, heuristic_t& heur, double epsilon,
             SccAlgorithm scc_alg = SccAlgorithm::ITE_TARJAN);
  ~PlannerTVI() { }


  /*
   * Planner Interface
   */
  action_t const* decideAction(state_t const& s) override {
    solve();
    return Bellman::constGreedyAction(s, v_, ssp_);
  }

  action_t const* decideAction(state_t const& s) const override {
    return Bellman::constGreedyAction(s, v_, ssp_);
  }

  void trainForUsecs(uint64_t max_time_usec) override {
    if (!runForUsec(max_time_usec, [this] { solve(); })) {
      std::cout << "[TVI::trainForUsecs]: training finished before convergence."
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
  double value(state_t const& s) const override { return v_.value(s); }

  /*
   * Optimal Planner Interface
   */
  double optimalSolution() override {
    solve();
    return v_.value(ssp_.s0());
  }


  /*
   * TVI main method.
   */
  // Main method for the PlannerTVI that call the static solve and update the
  // solved_ flag.
  void solve() {
    if (!solved_) {
      solve(ssp_, v_, epsilon_, scc_alg_);
      solved_ = true;
    }
  }

  /*
   * This method is static because there is no internal state for TVI,
   * thus, other planners can just use it without creating a PlannerTVI object.
   */
  static void solve(SSPIface const& ssp, hash_t& v, double epsilon,
                    SccAlgorithm scc_alg = SccAlgorithm::ITE_TARJAN)
  {
    std::vector<HashsetState> sorted_sccs =
                          stronglyConnectedComponentsInPostOrder(ssp, scc_alg);
    size_t idx = 0;
    for (auto const& scc : sorted_sccs) {
      idx++;
      _D(DEBUG_TVI,
        std::cout << idx << "-th SSC" << std::endl;
        for (auto const& s : scc) {
          std::cout << "\t" << s.toStringFull(gpt::problem, false)
                    << std::endl;
        }
        std::cout << "Calling VI..." << std::endl;
      );


      // This huge definition is here in order to take advantage of 'auto',
      // lambda and template deduction
      auto ssp_from_scc = makeSSPAdaptor(ssp, "SSP from SCC",
        // Any state in scc can be the new initial state because scc represents
        // a strongly-connected component, therefore from any state s in scc,
        // all the other states can be reachable from s.
        *scc.begin(),
        // The goals are any state that is outside the scc and the original
        // goals, thus returning true (i.e., makes s a new goal) if s is not in
        // scc, otherwise returns false (i.e., delegate to the original SSP goal
        // definition)
        [&scc](state_t const& s) -> bool { return scc.find(s) == scc.end(); },
        // Same as before
        SameApplicableActions(),
        // Same as before
        SameActionCost(),
        // The terminal cost is the cost from the value function of the
        // non-terminal states outside the scc or the original terminal cost for
        // the real goals.
        [&ssp, &v](state_t const& s) -> Rational {
          if (ssp.isGoal(s)) return ssp.terminalCost(s);
          else return Rational(v.value(s));
        },
        // Using the already defined scc.
        PrecomputedReachableStates(scc));

      PlannerVI::solve(ssp_from_scc, v, epsilon);
      gpt::incCounterAndCheckDeadlineEvery(idx, 1);
    }
  }


  /*
   * Strongly Connected Components methods
   *
   *
   * Return the strongly connected components of the graph G (detailed next)
   * in post-order (reverse topological order), i.e., the scc at position 0
   * does not have any outgoing arc and an scc at position i might have
   * outgoing arcs to any scc in the positions i-1,...,0 (top of the stack).
   *
   * The graph considered is G = (V,E), where V = S (state space) and E =
   * {(s,s') | exists a in A(s) \setminus ignore_action, P(s'|s,a) > 0}, i.e.,
   * the all-outcomes graph in which repeated arcs are removed as well as the
   * arcs forbidden by ignore_action.
   */
  static std::vector<HashsetState> stronglyConnectedComponentsInPostOrder(
      SSPIface const& ssp, SccAlgorithm scc_alg)
  {
    std::vector<HashsetState> scc;
    switch (scc_alg) {
     case SccAlgorithm::KOSARAJU:
      std::cout << "[TVI]: Using KOSARAJU's algorithm to find SCCs." << std::endl
        << "  There is a chance that it segfaults because of the stack size.\n"
        << "  In this case, try running 'ulimit -s unlimited' to make the "
        << "stack size 'unlimited'" << std::endl;
      scc = kosarajuStronglyConnectedComponentsInPostOrder(ssp);
      break;

     case SccAlgorithm::REC_TARJAN:
      std::cout << "[TVI]: Using Recursive TARJAN's algorithm to find SCCs\n"
        << "  There is a chance that it segfaults because of the stack size.\n"
        << "  In this case, try running 'ulimit -s unlimited' to make the "
        << "stack size 'unlimited'" << std::endl;
      scc = recTarjanStronglyConnectedComponentsInPostOrder(ssp);
      break;

     case SccAlgorithm::ITE_TARJAN:
      std::cout << "[TVI]: Using Iterative Tarjan's algorithm to find SCCs"
                << std::endl;
      scc = iteTarjanStronglyConnectedComponentsInPostOrder(ssp);
      break;
    }

    std::cout << "[TVI]: Number of SCCs = " << scc.size() << std::endl;

#ifdef TVI_DEBUG_ORDERED_SCCS
    std::cout << "[TVI]: Double checking if the SCCs are in the right order"
              << std::endl;
    if (debugStronglyConnectedComponentsInPostOrderRecursion(ssp, scc)) {
      std::cout << "[TVI]: SCCs are correct." << std::endl;
    }
    else {
      std::cout << "[TVI]: ERROR IN THE SCCs!" << std::endl;
      exit(-70);
    }
#endif
    return scc;
  }

  static
  std::vector<HashsetState> kosarajuStronglyConnectedComponentsInPostOrder(
                                                          SSPIface const& ssp);


  /*
   * Implementation of the Tarjan algorithm to find SCCs in the all-outcomes
   * graph (ignoring the actions pointed by ignore_action). See the comment of
   * stronglyConnectedComponentsInPostOrder for more details.
   *
   * Since we are dealing with SSPs, i.e., we only care about the reachable
   * states from s0, the Tarjan algorithm starts by looking at s0 and finish
   * when it has visited all the reachable states. In order words, it will not
   * find non-reached vertices and restart the search from them.
   */
  static
  std::vector<HashsetState> recTarjanStronglyConnectedComponentsInPostOrder(
                                                          SSPIface const& ssp);
  // Iterative version of the above algorithm
  static
  std::vector<HashsetState> iteTarjanStronglyConnectedComponentsInPostOrder(
                                                          SSPIface const& ssp);


 private:
  /*
   * Typedefs
   */
  // Hash to represent index and lowindex
  typedef std::unordered_map<state_t, std::pair<size_t, size_t>, hashState>
                                                          HashStateToPairSizeT;

  /*
   * Private methods
   */

  /*
   * Kosaraju SCC Helper methods
   * Returns the post-order DFS traversal of the graph as a list of hashEntry_t
   * This method is used by Kosaraju algorithm only.
   */
  static ListOfStates depthFirstSearchPostOrder(SSPIface const& ssp,
      state_t const& s);
  // Internal method for the recursion of the method above
  static void depthFirstSearchPostOrderRecursive(SSPIface const& ssp,
      state_t const& s, HashsetState& visited_nodes, ListOfStates& closed_nodes,
      size_t depth);

  /*
   * Recursive Tarjan SCC Helper method
   */
  static void recTarjanStronglyConnectedComponentsInPostOrderRecursion(
      SSPIface const& ssp, state_t const& s, size_t& index,
      std::stack<state_t>& stack,  HashStateToPairSizeT& indexAndLowindex,
      HashsetState& isInStack, std::vector<HashsetState>& ordered_sccs,
      size_t depth);


  /*
   * SCC debug method.
   */
  static bool debugStronglyConnectedComponentsInPostOrder(
      SSPIface const& ssp, std::vector<HashsetState> const& sccs);

  /*
   * Member variables
   */
  SSPIface const& ssp_;
  hash_t v_;
  double epsilon_;
  SccAlgorithm scc_alg_;
  bool solved_;
  ProbDistState pr_;
};

#endif  // PLANNER_TVI_H
