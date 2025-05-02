#ifndef PLANNER_FACTORY_H
#define PLANNER_FACTORY_H

#include <iostream>

#include "planner_iface.h"
#include "../heuristics/heuristic_cssp_iface.h"

Planner* createPlanner(ConstrSSPIface const& cssp, std::string const& name,
                        HeuristicCSSPUniqPtr h_vector);
Planner* createPlanner(SSPIface const& ssp, std::string const& name);

#endif // PLANNER_H
