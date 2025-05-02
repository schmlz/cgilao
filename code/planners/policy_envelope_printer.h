#ifndef ILAO_TYPES_H
#define ILAO_TYPES_H

static void dumpPolicyEnvelopeInfo(Planner const& planner, SSPIface const& ssp)
{
  std::unordered_set<state_t> seen_states;
  std::vector<state_t> frontier;
  seen_states.emplace(ssp.s0());
  frontier.emplace_back(ssp.s0());
  while (frontier.size() > 0) {
    const state_t s = frontier.back();
    frontier.pop_back();

    try {
      action_t const* a_ptr = planner.decideAction(s);
      if (a_ptr == nullptr) { continue; }

      ProbDistStateHash pr;
      ssp.expand(*a_ptr, s, pr);

      for (auto const& ip : pr) {
        state_t const& s_prime = ip.event();
        if (ssp.isGoal(s_prime)) { continue; }
        if (seen_states.find(s_prime) != seen_states.end()) { continue; }
        frontier.emplace_back(s_prime);
        seen_states.insert(s_prime);
      }
    } catch (const std::exception& e) {
    }
  }

  // This computes the number of q-values required to extract greedy policy
  // -- should be a lower bound on the number of q-values required in an algorithm
  size_t sum_actions_in_env = 0;
  for (state_t const& s : seen_states) {
    for (auto const& a : ssp.applicableActions(s)) {
      std::ignore = a;
      ++sum_actions_in_env;
    }
  }

  // CSV_POLICY_ENVELOPE_INFO,[# relevant states],[sum of actions from relevant states]
  std::cout << "CSV_POLICY_ENVELOPE_INFO," << seen_states.size() << "," << sum_actions_in_env
            << std::endl;
}

#endif  // ILAO_TYPES_H