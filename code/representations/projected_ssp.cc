#include <iostream>
#include <unordered_map>

#include "projected_ssp.h"

#if defined USE_GUROBI

#include "gurobi_c++.h"  // In the include path



void SasVariableProjectedSSP::buildProjectedActionVector() {
  std::vector<SasVarValuePr> v_pr;
  for (SasVarValue const& v : proj_var_.domain()) {
    for (PrSasAction const& a : sas_problem_.actions()) {
      if (isApplicable(v, a)) {
        expand(a, v, v_pr);
        assert(v_pr.size() > 0);
        if (v_pr.size() == 1) {
          actions_[v].emplace_back(a.name(), a, v_pr[0]);
        }
        else {
          for (size_t i = 0; i < v_pr.size(); ++i) {
            actions_[v].emplace_back(a.name() + "-" + std::to_string(i), a, v_pr[i]);
          }
        }
      }
    }
  }
}



HashValEnumActionPtrToGRBVar SasVariableProjectedSSP::buildDualFlowConstraints(
    GRBModel& model, int cost_idx_for_obj_func, bool add_sink_variable) const
{
  SasVarValue const& source = s0();

  HashValEnumActionPtrToGRBVar occ_measure;

  assert(cost_idx_for_obj_func < 0 ||
         cost_idx_for_obj_func < (int) sas_artificial_action_->costVector().size());
  /*
   * Building all the occupation measures, even if they are unreachable. This
   * because the problem will not be generated for each different source,
   * instead, it will be reused, so the occupation measure should be there.
   */
  for (auto const& pair : actions_) {
    SasVarValue const& v = pair.first;
    for (EnumProjectedAction const& a : pair.second) {
      // By definition, a is applicable in v
      double obj_f_coeff = 0.0;
      if (cost_idx_for_obj_func >= 0) {
        obj_f_coeff = a.costVector()[cost_idx_for_obj_func].double_value();
      }
      occ_measure[{v,&a}] = model.addVar(0, GRB_INFINITY, obj_f_coeff,
                                         GRB_CONTINUOUS, omName(v,a));
    }
  }
  if (add_sink_variable) {
    occ_measure[{artificialGoal(), nullptr}] = model.addVar(0, GRB_INFINITY, 0.0,
                                                            GRB_CONTINUOUS,
                                                            "sink_" + proj_var_.name());
  }

  // Integrate new variables
  model.update();

  // flow will be represented as OUT(v) - IN(v)
  using HashValueToLinExpr = std::unordered_map<SasVarValue, GRBLinExpr>;
  HashValueToLinExpr flow_constr_lhs;

  for (SasVarValue const& v : proj_var_.domain()) {
    // totalActions also takes into consideration the artificial_action_
    auto const applicable_actions_in_v = actions_.find(v);
    if (applicable_actions_in_v == actions_.end())
      continue;

    for (EnumProjectedAction const& a : applicable_actions_in_v->second) {
      assert(occ_measure.find({v,&a}) != occ_measure.end());
      GRBVar const& x_v_a = occ_measure[{v,&a}];
      // x_{v,a} is in OUT(v)
      flow_constr_lhs[v] += x_v_a;

      for (auto const& it : a.eff()) {
        SasVarValue const& v_prime = it.first;
        double const& p = it.second;
        flow_constr_lhs[v_prime] -= p * x_v_a;
      }
    }  // a \in A(v)
  }  // for all v \in domain of V

#if not defined NDEBUG
  for (SasVarValue const& v : proj_var_.domain()) {
    assert(flow_constr_lhs.find(v) != flow_constr_lhs.end());
  }
  assert(flow_constr_lhs.find(artificial_goal_) != flow_constr_lhs.end());
#endif

  /*
   * Adding the flow constraints
   */
  for (auto const& it : flow_constr_lhs) {
    SasVarValue const& v = it.first;
    GRBLinExpr const& lhs = it.second;
    double rhs = 0.0;
    if (v == source) {
      rhs = 1.0;
    }
    else if (v == artificial_goal_) {
      rhs = -1.0;
      if (add_sink_variable) {
        assert(occ_measure.find({v,nullptr}) != occ_measure.end());
        // We need to multiply by -1 because the in-flow is always negative
        model.addConstr(occ_measure[{v,nullptr}], GRB_EQUAL, -1 * lhs);
      }
    }
    if (v != artificial_goal_ || !add_sink_variable) {
      assert(!isFlowConstrNameDefined(v, model));
      model.addConstr(lhs, GRB_EQUAL, rhs, flowConstrName(v));
    }
  }

  return occ_measure;
}


bool SasVariableProjectedSSP::isFlowConstrNameDefined(SasVarValue const& v,
    GRBModel& model) const
{
  // Need to update the model to make sure all the previously added constraints
  // are available for search
  model.update();
  bool constr_name_is_defined = true;
  try {
    model.getConstrByName(flowConstrName(v));
  }
  catch (GRBException const& e) {
    constr_name_is_defined = false;
  }
  return constr_name_is_defined;
}



void SasVariableProjectedSSP::dump() const {
  std::cout << "name = " << name()
            << "\ns0 = " << s0()
            << "\nGoal = " << artificial_goal_
            << "\nStates:\n";
  for (SasVarValue const& v : proj_var_.domain()) {
    std::cout << "  v = " << v << std::endl
              << "  Applicable actions:";
    auto const it = actions_.find(v);
    if (it == actions_.end()) {
      std::cout << " NONE!\n";
      continue;
    }
    std::cout << " [" << it->second.size() << "]" << std::endl;
    for (EnumProjectedAction const& a : it->second) {
      std::cout << "    " << a.name() << " -- COST = [";
      for (Rational const& c : a.costVector()) {
        std::cout << c << " ";
      }
      std::cout << "]\n";
      for (auto const& ip : a.eff()) {
        std::cout << "      " << ip.second << ": " << ip.first << std::endl;
      }
    }
  }
}

#endif  // USE_GUROBI