#include "greedy.h"

PlannerGreedy::PlannerGreedy(SSPIface const& ssp, hash_t const& v)
  : HeuristicPlanner(), ssp_(ssp), internal_v_(nullptr), v_(v)
{ }

PlannerGreedy::PlannerGreedy(SSPIface const& ssp, heuristic_t& heur)
  : HeuristicPlanner(), ssp_(ssp),
    internal_v_(new hash_t(gpt::initial_hash_size, heur)),
    v_(*internal_v_)
{ }
