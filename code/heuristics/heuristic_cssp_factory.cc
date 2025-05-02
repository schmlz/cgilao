#include <iostream>
#include <sstream>

#include <boost/algorithm/string.hpp>

#include "heuristic_cssp_factory.h"

#include "dead-end.h"
#include "heuristic_cssp_detAtom.h"
#include "heuristic_cssp_iface.h"
#include "max-of.h"

#include "../ext/mgpt/global.h"

HeuristicCSSPUniqPtr buildHeuristicCSSP(ConstrSSPIface const& cssp,
    std::string const& full_name)
{
  std::deque<std::string> tokens;
  gpt::json_output.overwrite("cssp heur", full_name);
  boost::split(tokens, full_name, boost::is_any_of(":"));

  if (tokens.empty()) {
    std::cout << "[C-SSP Heuristic factory] No heuristic was given. Quitting" << std::endl;
    exit(-1);
  }

  HeuristicCSSPUniqPtr h = buildHeuristicCSSP(cssp, tokens);

  if (!tokens.empty()) {
    std::cout << "[Heuristic factory] WARNING: Unused heuristic tokens: "
              << boost::join(tokens, ":") << std::endl;
  }
  return h;
}

HeuristicCSSPUniqPtr buildHeuristicCSSP(ConstrSSPIface const& cssp,
    std::deque<std::string>& tokens)
{
  std::string name(tokens[0]);
  tokens.pop_front();

  if (boost::iequals(name, "smartZero") || boost::iequals(name, "zero")) {
    return HeuristicCSSPUniqPtr(new SmartZeroVector(cssp));
  }
  else {
    // FIXME HACK TODO(fwt)
    problem_t const& problem = *gpt::problem;

  }

  std::cerr << "[buildHeuristicCSSP] Unknown C-SSP heuristic '"
            << name << "'. Quitting."
            << std::endl;
  exit(-54);
  return HeuristicCSSPUniqPtr();
}
