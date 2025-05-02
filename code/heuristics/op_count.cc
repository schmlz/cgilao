#include <iostream>

#include "op_count.h"

#include "../representations/pr_sas_iface.h"

template<typename CG>
void OperatorCountTemplate<CG>::buildOperatorPartition() {
  std::cout << "[OperatorCount] Building operator partition\n";
  for (auto const& variable : sas_problem_sptr_->variables()) {
    for (auto const& val : variable.domain()) {

      /*
       * ASSUMPTION(fwt): the value of each variable is unique, i.e.:
       *            |union_V domain_V| = sum_V |domain_V|
       * This holds for the HackedPrSasProblem since each value correspond to an
       * atom id; however, this will not hold in general.
       */
      DIE(op_partition_.find(val) == op_partition_.end(),
          "ASSUMPTION about the uniqueness of the variables domain doesn't hold", -1);

      OperatorPartition& part_val = op_partition_[val];
      auto const& actions = sas_problem_sptr_->actions();
      for (size_t ai = 0; ai < actions.size(); ++ai) {
        NonConditionalPrSasAction const& a = actions[ai];
        SasValuation const& prec = a.prec();

        for (size_t ei = 0; ei < a.size(); ++ei) {
          SasValuation const& eff = a.eff(ei);
          assert(eff_to_grbvar_.find(&eff) != eff_to_grbvar_.end());
          LPWrap::Var& grbvar_for_eff = eff_to_grbvar_[&eff];

          auto eff_it = eff.find(variable);
          auto prec_it = prec.find(variable);

          if (eff_it != eff.end()) {
            if (eff_it->second == val) {
              // a might produce V = v
              if (prec_it == prec.end()) {
                part_val.sometimes_produce.push_back(&grbvar_for_eff);
              }
              else if (prec_it->second != val) {
                part_val.always_produce.push_back(&grbvar_for_eff);
              }
            }
            else {
              // a might consume V = v
              if (prec_it == prec.end()) {
                part_val.sometimes_consume.push_back(&grbvar_for_eff);
              }
              else if (prec_it->second == val) {
                part_val.always_consume.push_back(&grbvar_for_eff);
              }
            }
          }
        }  // for all probabilistic effects of action a
      }  // for all actions a
    }  // for all value for the SAS+ variable
  }  // for all SAS+ variable
  std::cout << "[OperatorCount] Done\n";
}


template<typename CG>
std::pair<int,int> OperatorCountTemplate<CG>::possibleNetChangeFrom(state_t const& s,
      SasVariable const& variable, SasVarValue const& value) const
{
  SasValuation const& goal = sas_problem_sptr_->goal();
  auto goal_it = goal.find(variable);
  if (goal_it == goal.end()) {
    // s_G[V] = \bot
    if (variable.valueAt(s) != value) { return {0,1}; }
    else                              { return {-1,0}; }
  }
  else if (goal_it->second == value && variable.valueAt(s) != value) {
    return {1,1};
  }
  else if (goal_it->second != value && variable.valueAt(s) == value) {
    return {-1,-1};
  }
  return {0,0};
}


template<typename CG>
void OperatorCountTemplate<CG>::buildModelFor(state_t const& s) {
  // This method should be called just once for a given model. Once the model is
  // built, it should by changed using changeModelFor or deleted and build
  // again.
  assert(op_partition_.empty());
  assert(used_constr_names_.empty());
  assert(model_built_for_.empty());

  // Minimize the sum of Y_a C(a) for all a in A(s). This is done using the
  // coefficient in objective function defined when the LPWrap::Vars get built below.
  auto row = model_->row();
  model_->set_minimisation_objective(row);


  /*
   * Building variables and objective
   */
#if defined USE_CPLEX
    // HAVE TO ADD VARS TO CPLEX IN BATCH, OTHERWISE IT'S TOO SLOW

    // count num vars
    size_t num_vars = 0;
    for (auto const& a : sas_problem_sptr_->actions()) {
      num_vars += a.size();
    }
    // initialise arrays
    auto vars = model_->get_var_array(num_vars);
    auto obj_vals = model_->get_float_array(num_vars);
    // bind vars to map and populate obj_vals
    size_t var_idx = 0;
    for (auto const& a : sas_problem_sptr_->actions()) {
      double action_cost = a.costVector()[cost_idx_for_obj_func_].double_value();
      for (size_t ei = 0; ei < a.size(); ++ei) {
        SasValuation const& eff = a.eff(ei);
        eff_to_grbvar_[&eff] = vars[var_idx];
        obj_vals[var_idx] = action_cost;
        var_idx += 1;
      }
    }
    // assign obj_vals to vars
    model_->update_signs_in_objectives(vars, obj_vals);
#endif
#if defined USE_GUROBI
  for (auto const& a : sas_problem_sptr_->actions()) {
    double action_cost = a.costVector()[cost_idx_for_obj_func_].double_value();
    for (size_t ei = 0; ei < a.size(); ++ei) {
      SasValuation const& eff = a.eff(ei);
      // eff_to_grbvar_[&eff] = model_->addVar(0,                // lower bound
      //                                       GRB_INFINITY,     // upper bound
      //                                       action_cost,      // obj func coef
      //                                       GRB_CONTINUOUS);  // var type
      eff_to_grbvar_[&eff] = model_->add_variable(0,                // lower bound
                                                  std::numeric_limits<double>::infinity(),     // upper bound
                                                  action_cost);     // obj func coef
    }
  }
  model_->update();
#endif

  buildOperatorPartition();

  /*
   * Building the constraints
   */
  try {
    for (auto const& variable : sas_problem_sptr_->variables()) {
      model_built_for_[variable] = variable.valueAt(s);

      for (auto const& val : variable.domain()) {

        assert(op_partition_.find(val) != op_partition_.end());
        OperatorPartition const& op_part = op_partition_.find(val)->second;

        // Compute the pnc^{s -> *}_{V=v} for each variable and value v
        std::pair<int,int> lb_and_ub = possibleNetChangeFrom(s, variable, val);

        // sum_{o in AP{V=v} U SP{V=v}} Y_o - sum_{o in AC{V=v}} Y_o >= min pnc{V=v}
        LPWrap::Row lb_constr = model_->row();
        if (op_part.always_produce.size() > 0
            || op_part.always_consume.size() > 0
            || op_part.sometimes_produce.size() > 0)
        {
          for (LPWrap::Var const* y_ae : op_part.always_produce)
            { lb_constr += *y_ae; }
          for (LPWrap::Var const* y_ae : op_part.sometimes_produce)
            { lb_constr += *y_ae; }
          for (LPWrap::Var const* y_ae : op_part.always_consume)
            { lb_constr -= *y_ae; }
          std::string constr_name("LB_" + to_string(val));
          // model_->addConstr(lb_constr, GRB_GREATER_EQUAL, lb_and_ub.first, constr_name);
          mutable_constraints_[constr_name] = model_->add_constraint(lb_constr >= lb_and_ub.first, constr_name);
          // True if constr_name was inserted, i.e., was not defined before
          assert(used_constr_names_.insert(constr_name).second);
        }

        // sum_{o in AP{V=v}} Y_o - sum_{o in AC{V=v} U SC{V=v}} Y_o <= max pnc{V=v}
        if (op_part.always_produce.size() > 0
            || op_part.always_consume.size() > 0
            || op_part.sometimes_consume.size() > 0)
        {
          LPWrap::Row ub_constr = model_->row();
          for (LPWrap::Var const* y_ae : op_part.always_produce)
            { ub_constr += *y_ae; }
          for (LPWrap::Var const* y_ae : op_part.always_consume)
            { ub_constr -= *y_ae; }
          for (LPWrap::Var const* y_ae : op_part.sometimes_consume)
            { ub_constr -= *y_ae; }
          std::string constr_name("UB_" + to_string(val));
          // model_->addConstr(ub_constr, GRB_LESS_EQUAL, lb_and_ub.second, constr_name);
          mutable_constraints_[constr_name] = model_->add_constraint(ub_constr <= lb_and_ub.second, constr_name);
          // True if constr_name was inserted, i.e., was not defined before
          assert(used_constr_names_.insert(constr_name).second);
        }
      }  // for all value for the SAS+ variable
    }  // for all sas+ variable
    model_->update();
  }
  catch (...) {
    std::cout << "Something went wrong with LP Solver...";
    exit(220);
  }
  // catch (GRBException const& e) {
  //   std::cout << "GRBException caught while building the new model. Error code = "
  //             << e.getErrorCode()
  //             << std::endl
  //             << e.getMessage() << std::endl
  //             << "For now quitting..." << std::endl;
  //   exit(220);
  // }
}


template<typename CG>
void OperatorCountTemplate<CG>::changeModelFor(state_t const& s) {
  // Instead of looping over all the variables and their possible values to
  // change the constraints, we need to do it only over the variables that
  // changed from the previous state (model_built_for_) to the current one (s)
  // and their previous value and current value.
  for (auto const& variable : sas_problem_sptr_->variables()) {
    assert(model_built_for_.find(variable) != model_built_for_.end());
    SasVarValue const& old_val = model_built_for_[variable];
    SasVarValue const& val_on_s = variable.valueAt(s);
    if (old_val != val_on_s) {
      changeConstraintFor(s, variable, old_val);
      changeConstraintFor(s, variable, val_on_s);
      // Updating since the model now is for a different state
      model_built_for_[variable] = val_on_s;
    }
  }
  model_->update();
}


template<typename CG>
void OperatorCountTemplate<CG>::changeConstraintFor(state_t const& s,
    SasVariable const& variable, SasVarValue const& val)
{
  /*
   * Changing the RHS of the constraints of V = v (variable = val).
   */
  // Compute the pnc^{s -> *}_{V=v} for each variable and value v
  std::pair<int,int> lb_and_ub = possibleNetChangeFrom(s, variable, val);

  assert(op_partition_.find(val) != op_partition_.end());
  OperatorPartition const& op_part = op_partition_.find(val)->second;

  try {
    // sum_{o in AP{V=v} U SP{V=v}} Y_o - sum_{o in AC{V=v}} Y_o >= min pnc{V=v}
    if (op_part.always_produce.size() > 0
        || op_part.always_consume.size() > 0
        || op_part.sometimes_produce.size() > 0)
    {
      std::string constr_name("LB_" + to_string(val));
      assert(used_constr_names_.find(constr_name) != used_constr_names_.end());
      // model_->getConstrByName(constr_name).set(GRB_DoubleAttr_RHS, lb_and_ub.first);
#if defined USE_GUROBI
      mutable_constraints_.at(constr_name).set(GRB_DoubleAttr_RHS, lb_and_ub.first);
#elif defined USE_CPLEX
      model_->set_constraint_lb(mutable_constraints_.at(constr_name), lb_and_ub.first);
#endif
    }

    // sum_{o in AP{V=v}} Y_o - sum_{o in AC{V=v} U SC{V=v}} Y_o <= max pnc{V=v}
    if (op_part.always_produce.size() > 0
        || op_part.always_consume.size() > 0
        || op_part.sometimes_consume.size() > 0)
    {
      std::string constr_name("UB_" + to_string(val));
      assert(used_constr_names_.find(constr_name) != used_constr_names_.end());
      // model_->getConstrByName(constr_name).set(GRB_DoubleAttr_RHS, lb_and_ub.second);
#if defined USE_GUROBI
      mutable_constraints_.at(constr_name).set(GRB_DoubleAttr_RHS, lb_and_ub.second);
#elif defined USE_CPLEX
      model_->set_constraint_ub(mutable_constraints_.at(constr_name), lb_and_ub.second);
#endif
    }
  }
  catch (...) {
    std::cout << "Something went wrong with LP Solver...";
    exit(220);
  }
  // catch (GRBException const& e) {
  //   std::cout << "GRBException caught while changing constraint. Error code = "
  //             << e.getErrorCode()
  //             << std::endl
  //             << e.getMessage() << std::endl
  //             << "For now quitting..." << std::endl;
  //   exit(220);
  // }
}


template<typename CG>
double OperatorCountTemplate<CG>::solveModel() {
  model_->solve();
  if (model_->solve_status() == SolveStatus::INFEASIBLE) {
    // For this particular problem, deterministic operator count, an
    // infeasible problem means that the current state is a dead-end.
    return gpt::dead_end_value.double_value();
  } else if (model_->solve_status() != SolveStatus::OPTIMAL) {
    std::cout << "[lp-dual]: LP Solver DID NOT find the optimal solution. " << std::endl
              << "Quitting" << std::endl;
    exit(181);
  }

  double solution = model_->get_objective();
  return solution;

  // try {
  //   model_->solve();
  //   int gurobi_status = model_->get(GRB_IntAttr_Status);
  //   if (gurobi_status == GRB_INFEASIBLE) {
  //     // For this particular problem, deterministic operator count, an
  //     // infeasible problem means that the current state is a dead-end.
  //     return gpt::dead_end_value.double_value();
  //   }
  //   else if (gurobi_status != GRB_OPTIMAL) {
  //     std::cout << "[lp-dual]: Gurobi DID NOT find the optimal solution. "
  //               << "GRB_IntAttr_Status = " << gurobi_status << std::endl
  //               << "Gurobi's error desc = '"
  //               << Gurobi::errorCodeTranslation(gurobi_status)
  //               << "'" << std::endl << "Quitting" << std::endl;
  //     exit(181);
  //   }
  //   double solution = model_->get(GRB_DoubleAttr_ObjVal);
  //   return solution;
  // }
  // catch (GRBException const& e) {
  //   std::cout << "GRBException caught. Error code = " << e.getErrorCode()
  //     << std::endl
  //     << e.getMessage() << std::endl
  //     << "For now quitting..." << std::endl;
  //   exit(220);
  // }
}

template class OperatorCountTemplate<OpCountConstraintsGenerator::NoExtraConstraint>;
template class OperatorCountTemplate<OpCountConstraintsGenerator::RegroupConstraint>;
template class OperatorCountTemplate<OpCountConstraintsGenerator::CostConstraintRegrouped>;
