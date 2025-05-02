#include <cassert>
#include <math.h>
#include <set>
#include <stdlib.h>

#include "planner_iface.h"
#include "rtdp.h"

#include "../ssps/bellman.h"
#include "../utils/die.h"
#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ext/mgpt/states.h"




/*******************************************************************************
 *
 * planner RTDP: Real Time Dynamic Programming
 *
 ******************************************************************************/

PlannerRTDP::PlannerRTDP(SSPIface const& ssp, heuristic_t& heur,
    double epsilon)
  : HeuristicPlanner(), ssp_(ssp), v_(gpt::initial_hash_size, heur), epsilon_(epsilon)
{ }


void PlannerRTDP::trial() {
#ifdef DEBUG_RTDP
    std::cout << "<trial>" << std::endl;
#endif

  state_t cur_s = ssp_.s0();
  // Keeping a hashEntry_t around to save a few hash calls
  hashEntry_t* cur_node = v_.get(cur_s);
  size_t id = 0;

  while (true) {
    // This should always hold
    assert(cur_s == *(cur_node->state()));
    gpt::incCounterAndCheckDeadlineEvery(id, 1000);
#ifdef DEBUG_RTDP
      std::cout << "<Tturn><id>" << id << "</id><state>"
                << cur_s.toStringFull(gpt::problem)
                << "</state><value>" << cur_node->value()
                << "</value>";
#endif

    if (cur_node->value() >= gpt::dead_end_value.double_value()) {
#ifdef DEBUG_RTDP
      std::cout << "<dead-end /></Tturn>";
#endif
      break;
    }
    else if (ssp_.isGoal(cur_s)) {
#ifdef DEBUG_RTDP
      std::cout << "<goal-reached /></Tturn>";
#endif
      break;
    }
    // Get the greedy action and also the min Q-value of it when applied in the
    // cur_s state.
    action_t const* a_greedy = nullptr;
    std::tie(a_greedy, std::ignore) = Bellman::update(cur_s, v_, ssp_);

    if (a_greedy == nullptr) {
      DIE(!ssp_.isGoal(cur_s), "Expecting dead end, received a goal",
          149);
#ifdef DEBUG_RTDP
      std::cout << "<dead-end /></Tturn>";
#endif
      break;
    }

#ifdef DEBUG_RTDP
    std::cout << "<action>" << a_greedy->name() << "</action></Tturn>";
#endif

    a_greedy->affect(cur_s);  // Sampling the next state
    cur_node = v_.get(cur_s); // Moving the search
  }

#ifdef DEBUG_RTDP
    std::cout << "</trial>" << std::endl;
#endif
}


void PlannerRTDP::statistics(std::ostream& os, int level) const {
  if (level > 0) {
    os << "<rtdp>: hash size = " << v_.size() << std::endl;
    os << "<rtdp>: hash diameter = " << v_.diameter() << std::endl;
    os << "<rtdp>: hash dimension = " << v_.dimension() << std::endl;
  }

  if (level >= 300)
    v_.print(os, ssp_);
}

