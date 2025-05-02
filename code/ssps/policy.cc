#include <iostream>

#include "policy.h"

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/problems.h"

void fancyProbPolicyDebugRec(ProbPolicy& prob_pi, state_t const& s,
                             HashsetState& open_or_closed, bool partial_pi,
                             std::string indentation)
{
  if (gpt::problem->isGoal(s)) {
    std::cout << indentation << "GOAL: " << s.toStringFull(gpt::problem, false, true)
              << std::endl;
    open_or_closed.insert(s);
    return;
  }

  if (open_or_closed.find(s) != open_or_closed.end()) {
    std::cout << indentation << s.toStringFull(gpt::problem, false, true)
              << "  already printed" << std::endl;
    return;
  }

  open_or_closed.insert(s);

  assert(partial_pi || prob_pi.isDefinedFor(s));
  if (!prob_pi.isDefinedFor(s)) {
    std::cout << indentation << s.toStringFull(gpt::problem, false, true) << " -- undefined\n";
    return;
  }

  DistActionP const& pi_s = prob_pi[s];

  double k = pi_s.normalizingConstant();
  bool first = true;
  for (auto const& it : pi_s) {
    if (first) {
      std::cout << indentation << s.toStringFull(gpt::problem, false, true) << ",";
      first = false;
    }
    else {
      std::cout << indentation << "(prev. state),";
    }
    action_t const* a = it.event();

    if (!a || a->name() == std::string("fringe/d-e")) {
      std::cout << "nullptr" << " = " << (it.prob() / k) << std::endl;
      continue;
    }

    std::cout << a->name() << " = " << (it.prob() / k) << std::endl;

    ProbDistStateHash pr; // CANNOT BE STATIC BECAUSE OF RECURSION
    gpt::problem->expand(*a, s, pr);
    for (auto const& ip : pr) {
      fancyProbPolicyDebugRec(prob_pi, ip.event(), open_or_closed, partial_pi, indentation + "  ");
    }
  }
}

void fancyProbPolicyDebug(ProbPolicy& prob_pi, state_t const& s0) {
  HashsetState open_or_closed_states;
  std::cout << "\nPOLICY DUMP:\n";
  fancyProbPolicyDebugRec(prob_pi, s0, open_or_closed_states, false, "");
  std::cout << "\nDUMP FINISHED\n\n";
}

void fancyPartialProbPolicyDebug(ProbPolicy& prob_pi, state_t const& s0) {
  HashsetState open_or_closed_states;
  std::cout << "\nPARTIAL POLICY DUMP:\n";
  fancyProbPolicyDebugRec(prob_pi, s0, open_or_closed_states, true, "");
  std::cout << "\nDUMP FINISHED\n";
}
