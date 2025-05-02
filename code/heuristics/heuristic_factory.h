#ifndef HEURISTICS_FACTORY_H
#define HEURISTICS_FACTORY_H

#include <iostream>
#include <stack>

#include "heuristic_iface.h"
#include "../ssps/ssp_iface.h"


heuristic_t* createHeuristic(SSPIface const& ssp, std::string const& full_name);

heuristic_t* createHeuristic(SSPIface const& ssp, std::deque<std::string>& tokens);

inline void createGlobalHeuristic(SSPIface const& ssp, std::string const& full_name) {
  gpt::heur_ptr.reset(createHeuristic(ssp, full_name));
}

#endif  // HEURISTICS_FACTORY_H
