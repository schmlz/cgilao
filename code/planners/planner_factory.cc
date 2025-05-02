#include <assert.h>
#include <boost/algorithm/string/predicate.hpp>
#include <math.h>
#include <set>
#include <stdlib.h>

#include <boost/algorithm/string.hpp>

#include "planner_factory.h"

#include "cg-ilao.h"
#include "cg-ilao-extended.h"
#include "ftvi.h"
#include "greedy.h"
#include "ilao.h"
#include "lrtdp.h"
#include "random.h"
#include "rtdp.h"
#include "tvi.h"
#include "vi.h"

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../simulators/simulator.h"
#include "../ext/mgpt/states.h"
#include "../heuristics/h_max.h"


Planner* createPlanner(ConstrSSPIface const& cssp, std::string const& name,
                        HeuristicCSSPUniqPtr h_vector) {
  return nullptr;
}

Planner* createPlanner(SSPIface const& ssp, std::string const& name) {
  using boost::iequals;
  using boost::istarts_with;

  gpt::json_output.insert("ssp planner", name);

  if (istarts_with(name, "cg-ilao-extended")) {
    // No defaults to make sure we don't make the mistake of comparing runs with different defaults
    std::vector<std::string> opts{"", "", "", "", "", ""};
    std::deque<std::string> tokens;
    boost::split(tokens, name, boost::is_any_of(":"));
    tokens.pop_front();
    if (tokens.size() != opts.size()) {
      std::cout << "Wrong number of options for cg-dual-extended! Expecting " << opts.size()
                << " parameters\n";
      assert(false);
      exit(-1);
    }
    for (std::string& o : opts) {
      o = tokens.front();
      tokens.pop_front();
    }
    return new PlannerCGiLAOExtended(ssp, *gpt::heur_ptr, gpt::epsilon,
                                     opts[0], opts[1], opts[2], opts[3], opts[4], opts[5]);
  }

  else if (istarts_with(name, "cg-ilao") and not istarts_with(name, "cg-ilao-extended")) {
    return new PlannerCGiLAO(ssp, *gpt::heur_ptr, gpt::epsilon);
  }
  else if (iequals(name, "ilao")) {
    return new PlannerILAO(ssp, *gpt::heur_ptr, gpt::epsilon);
  }
  else if (iequals(name, "ftvi")) {
    return new PlannerFTVI(ssp, *gpt::heur_ptr, gpt::epsilon);
  }
  else if (istarts_with(name, "tvi")) {
    if (name.size() > 3) {
      std::string scc_alg_name = name.substr(4);
      if (scc_alg_name == "kosaraju") {
        return new PlannerTVI(ssp, *gpt::heur_ptr, gpt::epsilon,
                                 PlannerTVI::SccAlgorithm::KOSARAJU);
      }
      else if (scc_alg_name == "rec_tarjan") {
        return new PlannerTVI(ssp, *gpt::heur_ptr, gpt::epsilon,
                                 PlannerTVI::SccAlgorithm::REC_TARJAN);
      }
      else if (scc_alg_name == "ite_tarjan") {
        return new PlannerTVI(ssp, *gpt::heur_ptr, gpt::epsilon,
                                 PlannerTVI::SccAlgorithm::ITE_TARJAN);
      }
      else {
        std::cout << "SCC algorithm '" << scc_alg_name << "' not recognized. "
                  << "Quitting..." << std::endl;
        exit(-1);
      }
    }
    else
      // Use the default SCC Algorithm
      return new PlannerTVI(ssp, *gpt::heur_ptr, gpt::epsilon);
  }
  else if (iequals(name, "random")) {
    return new PlannerRandom(ssp);
  }
  else if (iequals(name, "vi")) {
    return new PlannerVI(ssp, *gpt::heur_ptr, gpt::epsilon);
  }
  else if (iequals( name, "rtdp"))  {
    return new PlannerRTDP(ssp, *gpt::heur_ptr, gpt::epsilon);
  }
  else
  if (iequals(name, "lrtdp"))  {
    return new PlannerLRTDP(ssp, *gpt::heur_ptr, gpt::epsilon,
                                MAX_TRACE_SIZE, false);
  }
  else if (iequals(name, "greedy")) {
    return new PlannerGreedy(ssp, *gpt::heur_ptr);
  }

//  std::cout << "[Warning] No SSP planner named '" << name << "'" << std::endl;
  return nullptr;
}
