#ifndef HEURISTICS_SSPPDB_FROM_MOSSP_H
#define HEURISTICS_SSPPDB_FROM_MOSSP_H

#include "heuristic_iface.h"

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/rational.h"
#include "../ext/mgpt/states.h"
#include "../utils/die.h"
#include "../ssps/ssp_iface.h"

#include <iostream>

#include "../ext/pdbs/ext/pr-sas/pr_sas_iface.h"
#include "../ext/pdbs/ssp_pdbs/canonical_ssp_pdbs_heuristic.h"
#include "../representations/pr_sas_iface.h"

class SSPPDBHeuristic : public heuristic_t {
 public:
  SSPPDBHeuristic(problem_t const& problem, std::string const& pattern_size_str, bool const use_dend_transform)
    : heuristic_t("SSP-PDB-H"), problem_(problem)
  {
    // Construct internal SAS representation (if needed)
    if (!gpt::cached_cond_sas_problem) {
      gpt::cached_cond_sas_problem.reset(new HackedPrSasProblem(problem, use_dend_transform));
    }
    if (!gpt::cached_non_cond_sas_problem) {
      gpt::cached_non_cond_sas_problem.reset(new NonConditionalPrSasProblem(*gpt::cached_cond_sas_problem, use_dend_transform));
    }
    assert(gpt::problem == &problem);
    assert(gpt::cached_cond_sas_problem->hasBeenDeadEndTransformed() == use_dend_transform);
    assert(gpt::cached_non_cond_sas_problem->hasBeenDeadEndTransformed() == use_dend_transform);

    // Construct SAS representation for PDB code
    translated_sas_problem_.reset(new SasPlus::SasPlusMOSSP(std::move(SasPlus::multiObjectiveSSPFromHackedPrSas(*gpt::cached_non_cond_sas_problem, translation_idxs_, translation_value_map_))));

    // Construct the external PDB heuristic
    size_t const pattern_size = std::stoi(pattern_size_str);
    pdb_heuristic_.reset(new ssp_pdbs::CanonicalPDBsHeuristic(*translated_sas_problem_, pattern_size));


    // std::cout << "\n\nh[pdb](s0) = " << computeValue(problem_.get_initial_state()) << "\n\n" << std::endl;
  }
  ~SSPPDBHeuristic() { }

  /*
   * heuristic_t interface
   */
  double computeValue(state_t const& s) {
    auto s_translated = SasPlus::translateState(s,
                                                *gpt::cached_non_cond_sas_problem,
                                                translation_idxs_,
                                                translation_value_map_);

    return pdb_heuristic_->heuristic(s_translated);
  }

 private:
  problem_t const& problem_;
  std::unordered_map<SasVariable, int> translation_idxs_;
  std::unordered_map<int, int> translation_value_map_;
  std::unique_ptr<SasPlus::SasPlusMOSSP> translated_sas_problem_;
  std::unique_ptr<ssp_pdbs::CanonicalPDBsHeuristic> pdb_heuristic_;
};



#endif // HEURISTICS_SSPPDB_FROM_MOSSP_H
