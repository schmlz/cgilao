#ifndef PDDL_TO_STRIPS_H__PDB
#define PDDL_TO_STRIPS_H__PDB

#include <iostream>

#include "problems.h"
#include "det_pddl_builder.h"

#include "../../representations/strips.h"

using STRIPS::StripsMOSSP;
using MdpsimProblem = PPDDL_PDB::Problem;

namespace PPDDL_PDB {


/// Parse a given PPDDL_PDB file containing both the domain and problem and returns the Multi-Object
/// STRIPS problem.
StripsMOSSP parseToStrips(std::string const& domain_and_problem_fname);

/// Parse the pair of PPDDL_PDB files (domain first and problem after) and problem and returns the
/// Multi-Object STRIPS problem.
StripsMOSSP parseToStrips(std::string const& domain_fname,
                                          std::string const& problem_fname);

/// Translate a given PPDDL_PDB::Problem to STRIPS::StripsMultiObjectiveProblem
StripsMOSSP translateToStrips(MdpsimProblem const& problem);

std::map<std::string, size_t> buildMetricToIdxMap(Problem const& problem);

}  // namespace PPDDL
#endif  // PDDL_TO_STRIPS_H_
