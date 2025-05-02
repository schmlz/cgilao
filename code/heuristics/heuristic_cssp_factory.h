#ifndef HEURISTIC_CSSP_FACTORY_H
#define HEURISTIC_CSSP_FACTORY_H

#include <memory>

#include "heuristic_cssp_iface.h"

HeuristicCSSPUniqPtr buildHeuristicCSSP(ConstrSSPIface const& cssp,
                                        std::string const& name);

HeuristicCSSPUniqPtr buildHeuristicCSSP(ConstrSSPIface const& cssp,
                                        std::deque<std::string>& tokens);


#endif  // HEURISTIC_CSSP_FACTORY_H
