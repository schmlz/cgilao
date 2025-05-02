#include <iostream>
#include <sstream>
#include <string.h>

#include <boost/algorithm/string.hpp>

#include "heuristic_factory.h"

#include "constant_value.h"
#include "dead-end.h"
#include "h_add.h"
#include "h_max.h"
#include "lm_cut.h"
#include "lrtdp_as_heuristic.h"
#include "max-of.h"
#include "omc.h"
#include "op_count.h"

#include "csv_heuristic.h"
#include "pdb.h"

#include "../utils/die.h"
#include "../ext/mgpt/global.h"


heuristic_t* createHeuristic(SSPIface const& ssp, std::string const& full_name) {
  std::deque<std::string> tokens;

  gpt::json_output.overwrite("ssp heur", full_name);
  boost::split(tokens, full_name, boost::is_any_of(":"));

  if (tokens.empty()) {
    std::cout << "[Heuristic factory] No heuristic was given. Quitting" << std::endl;
    exit(-1);
  }

  heuristic_t* h = createHeuristic(ssp, tokens);

  if (!tokens.empty()) {
    std::cout << "[Heuristic factory] WARNING: Unused heuristic tokens: "
              << boost::join(tokens, ":") << std::endl;
  }
  return h;
}

void failedToBuildHeuristic(std::string const& name) {
  std::cerr << "ERROR: not able to create heuristic '" << name
            << "'. Quitting" << std::endl;
  exit(-1);
}


heuristic_t* createHeuristic(SSPIface const& ssp, std::deque<std::string>& tokens) {
  /*
   * heuristic_t, i.e., SSPIface based heuristics
   */
  std::string name(tokens[0]);
  tokens.pop_front();

  heuristic_t* h = nullptr;

  if (boost::iequals(name, "max-of")) {
    if (tokens.size() == 0)
      failedToBuildHeuristic(name);
    bool prune_on_deadend = false;
    if (tokens[0] == "prune-deadend") {
      prune_on_deadend = true;
      tokens.pop_front();
    }
    VecSPtrHeuristic heur_vec;
    while (!tokens.empty()) {
      heur_vec.emplace_back(createHeuristic(ssp, tokens));
    }
    h = new MaxOfHeuristics(heur_vec, prune_on_deadend);
  }
  else if (boost::iequals(name, "simpleZero")) {
    h = new ZeroHeuristic();
  }
  else if (boost::iequals(name, "smartZero")) {
    h = new SmartZeroHeuristic(ssp);
  }
  else if (boost::starts_with(name, "CSV")) {
    std::deque<std::string> tokens;
    boost::split(tokens, name, boost::is_any_of(":"));
    h = new CSVHeuristic(tokens);
  }
  else {
    /*
     * FactoredHeuristic or descendants
     */
    // TODO(fwt): refactor the code to avoid this
    problem_t const& problem = *gpt::problem;

    if (boost::iequals(name, "dead-end")) {
      if (tokens.size() > 0) {
        h = new DeadendHeuristic(problem, createHeuristic(ssp, tokens));
      }
      else {
        h = new DeadendHeuristic(problem);
      }
    }
    else if (boost::iequals(name, "pdb2")) {
      h = new SSPPDBHeuristic(problem, "2", false);
    }
    else if (boost::iequals(name, "roc")) {
      if (tokens.size() > 0 && tokens[0] == "no-dead-end") {
        h = new RegroupedOperatorCount(problem, false);
        tokens.pop_front();
      }
      else {
        h = new RegroupedOperatorCount(problem, true);
      }
    }
    else if (boost::iequals(name, "lm-cut")) {
      h = new LMCutHeuristic(problem);
    }
    else if (boost::iequals(name, "lrtdp-as-heuristic")) {
      if (tokens.size() < 2) {
        std::cout << "Need to pass a scaling value and a normal heuristic to LRTDP-as-heuristic";
        assert(false);
        exit(-1);
      }
      const double scaling_value = std::stod(tokens.front());
      tokens.pop_front();
      h = new LRTDPHeuristic(ssp, *createHeuristic(ssp, tokens), scaling_value, gpt::epsilon);
    }
    else if (boost::iequals(name, "selfloop-h-add")) {
      h = new HAddSelfLoop(problem, ACTION_COST);
    }
    else if (boost::iequals(name, "selfloop-h-max")) {
      h = new HMaxSelfLoop(problem, ACTION_COST);
    }
    else if (boost::iequals(name, "h-add")) {
      h = new HAddAllOutcomesDet(problem, ACTION_COST);
    }
    else if (boost::iequals(name, "h-max")) {
      h = new HMaxAllOutcomesDet(problem, ACTION_COST);
    }


  }  // case in which the heuristic is a FactoredHeuristic

  if (!h) failedToBuildHeuristic(name);
  return h;
}
