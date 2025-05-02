#include <iostream>

#include "omc.h"

#include "../representations/projected_ssp.h"

#if defined USE_GUROBI

#include "gurobi_c++.h"  // In the include path

template<typename CG>
void OccupationMeasureCountTemplate<CG>::buildModelFor(state_t const& s) {
  /*
   * Building flow constraints for each projection
   */
  assert(proj_ssps_.size() == 0);
  assert(proj_occ_measure_.size() == 0);

  auto const& sas_variables = cond_sas_problem_sptr_->variables();
  for (size_t iv = 0; iv < sas_variables.size(); ++iv) {
    SasVariable const& var = sas_variables[iv];
    proj_ssps_.emplace_back(new SasVariableProjectedSSP(*cond_sas_problem_sptr_,
                                                        var, var.valueAt(s)));

    // Because of the tying constraints, any SAS+ variable can be used and the
    // result is the same. In this particular case, we are using the first var
    // (iv == 0) om's in the objective function
    proj_occ_measure_.push_back(proj_ssps_[iv]->buildDualFlowConstraints(*model_,
                                                (iv == 0 ? cost_idx_for_obj_func_ : -1)));
  }
  model_->set(GRB_IntAttr_ModelSense, 1);
  model_->update();


  /*
   * Building tying constraints
   */
  try {
    // TODO: HACK: FIXME: XXX: How about a_g (i.e., artificial_action_)????
    for (PrSasAction const& a : cond_sas_problem_sptr_->actions()) {
      GRBLinExpr x_a_var = proj_ssps_[0]->actionOM(a, *model_);
      // For every other SAS+ variable, building x^V'_a and adding the tying
      // constraint x^V_a = x^V'_a
      for (size_t iv = 1; iv < sas_variables.size(); ++iv) {
        GRBLinExpr x_a_var_prime = proj_ssps_[iv]->actionOM(a, *model_);
        model_->addConstr(x_a_var - x_a_var_prime, GRB_EQUAL, 0);
      }
    }
  }
  catch (GRBException const& e) {
    std::cout << "GrbException caught while optimizing. Error code = "
              << e.getErrorCode()
              << std::endl
              << e.getMessage() << std::endl
              << "For now quitting..." << std::endl;
    assert(false);
    exit(220);
  }
  model_->update();

  extra_constr_generator_.addExtraConstraints(s, *cond_sas_problem_sptr_, proj_ssps_,
      proj_occ_measure_, *model_);

  cur_model_for_ = s;
}


template<typename CG>
void OccupationMeasureCountTemplate<CG>::printGurobiSolution() const {
  int gurobi_status = model_->get(GRB_IntAttr_Status);
  if (gurobi_status == GRB_INFEASIBLE) {
    std::cout << "[grb solution] Current Gurobi problem is infeasible\n";
  }
  else if (gurobi_status != GRB_OPTIMAL) {
    std::cout << "[grb solution] Current Gurobi is neither optimal nor infeasible\n";
  }
  else {
    std::cout << "[grb solution] Current optimal solution:\n";
    for (size_t iv = 0; iv < cond_sas_problem_sptr_->variables().size(); ++iv) {
      for (auto const& ix : proj_occ_measure_[iv]) {
        std::cout << "  " << ix.second.get(GRB_StringAttr_VarName)
                  << " = " << ix.second.get(GRB_DoubleAttr_X) << std::endl;
      }
    }
  }
}

template class OccupationMeasureCountTemplate<
                                OccMeasCountConstraintsGenerator::NoExtraConstraint>;
template class OccupationMeasureCountTemplate<
                                OccMeasCountConstraintsGenerator::SelfLoopConstr>;
template class OccupationMeasureCountTemplate<
                                OccMeasCountConstraintsGenerator::CostConstraint>;

#endif  // USE_GUROBI