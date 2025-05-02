#ifndef EXT_PR_SAS_PR_SAS_IFACE_H__PDB
#define EXT_PR_SAS_PR_SAS_IFACE_H__PDB

#include <iostream>

#include "../mdpsim-parser/det_pddl_builder.h"
#include "../sas-parser/problem.h"
#include "../../representations/sasplus.h"

#include "../../../../representations/pr_sas_iface.h"

using PPDDL_PDB::DetPDDL_PDB;
using FastDwSasProblem_PDB = FastDownwardParser_PDB::SasProblem;

namespace SasPlus {

SasPlusMOSSP::State translateState(
    state_t const& s,
    NonConditionalPrSasProblem const& hack_sas_problem,
    std::unordered_map<SasVariable, int> const& idxs,
    std::unordered_map<int, int> const& value_map);

SasPlusMOSSP multiObjectiveSSPFromHackedPrSas(
  NonConditionalPrSasProblem const& hack_sas_problem,
  std::unordered_map<SasVariable, int> & idxs,
  std::unordered_map<int, int> & value_map);

SasPlusMOSSP multiObjectiveSSPFromPPDDL(std::string const& domain_fname,
                                        std::string const& problem_fname,
                                        std::string const& translate_py_path = "./translate.py");

FastDwSasProblem_PDB fastDownwardTranslateAndParse(DetPDDL_PDB const& det_pddl,
                                               std::string const& translate_bin);

SasPlusMOSSP translateToPrSas(PPDDL_PDB::Problem const& original_problem,
                              FastDwSasProblem_PDB const& det_sas_prob,
                              DetPDDL_PDB const& det_pddl);

void rebuildActions(PPDDL_PDB::Problem const& original_problem,
                    FastDwSasProblem_PDB const& det_sas_prob,
                    DetPDDL_PDB const& det_pddl,
                    std::vector<SasPlusVariable> const& variables,
                    std::unordered_map<FastDownwardParser_PDB::Variable*, size_t> var_ptr_to_idx,
                    std::vector<SasPlusAction>& actions);

void debug(FastDwSasProblem_PDB const& det_sas_prob, SasPlusMOSSP const& translated);

}  // namespace SasPlus

#endif  // PR_SAS_IFACE
