#ifndef HEURISTICS_OMC_H
#define HEURISTICS_OMC_H

#include <iostream>
#include <vector>
#include <unordered_set>

#include <boost/functional/hash.hpp>

#include "heuristic_iface.h"

#include "../ext/mgpt/global.h"
#include "../representations/pr_sas_iface.h"
#include "../representations/projected_ssp.h"
#include "../utils/gurobi.h"

#if defined USE_GUROBI

#include "gurobi_c++.h"  // In the include path

using VectorOfProjOM = std::vector<HashValEnumActionPtrToGRBVar>;

namespace OccMeasCountConstraintsGenerator {
  class NoExtraConstraint {
   public:
    static std::string name() { return "omc-simple"; }
    void addExtraConstraints(state_t const&,
                             HackedPrSasProblem const&,
                             std::vector<SasVariableProjectedSSPptr> const&,
                             VectorOfProjOM const&,
                             GRBModel& model)
    { }
    void updateExtraConstraints(state_t const&,
                                HackedPrSasProblem const&,
                                std::vector<SasVariableProjectedSSPptr> const&,
                                VectorOfProjOM const&,
                                GRBModel& model)
    { }
  };

  class SelfLoopConstr {
    using PairVarnameVal = std::pair<std::string, SasVarValue>;
    using HashsetPairVarnameVal = std::unordered_set<PairVarnameVal,
                                                   boost::hash<PairVarnameVal>>;

   public:
    SelfLoopConstr() : big_m_(-1.0) { }
    ~SelfLoopConstr() {
      std::cout << "[" << name() << "]: total self-loop constraints: "
                << constrs_.size() << std::endl;
    }

    static std::string name() { return "omc-selfloop-constr"; }

    void addExtraConstraints(state_t const& s,
                             HackedPrSasProblem const& sas_problem,
                             std::vector<SasVariableProjectedSSPptr> const& proj_ssps,
                             VectorOfProjOM const& proj_occ_measure,
                             GRBModel& model)
    {
      if (big_m_ < 0) {
        big_m_ = bigMConstantFromDeadendPenalty(sas_problem);
        std::cout << "[" << name() << "] Inferred big-M constant = " << big_m_
                  << std::endl;
      }
      assert(big_m_ > 0.0);
      auto const& sas_variables = sas_problem.variables();
      for (size_t iv = 0; iv < sas_variables.size(); ++iv) {
        addSelfLoopConstrForProjection(sas_variables[iv], *proj_ssps[iv],
                                       proj_occ_measure[iv], model,
                                       sas_variables[iv].valueAt(s));
      }
      cur_constrs_for = s;
    }

    void updateExtraConstraints(state_t const& s,
                                HackedPrSasProblem const& sas_problem,
                                std::vector<SasVariableProjectedSSPptr> const& proj_ssps,
                                VectorOfProjOM const& proj_occ_measure,
                                GRBModel& model)
    {
      assert(big_m_ > 0.0);
      for (SasVariable const& var : sas_problem.variables()) {
        SasVarValue const& old_val = var.valueAt(cur_constrs_for);
        SasVarValue const& new_val = var.valueAt(s);
        if (old_val != new_val) {
          // Turning on the self-loop constraint for old_val
          if (constrs_.find({var.name(), old_val}) != constrs_.end()) {
              model.getConstrByName(constrName(var, old_val)).set(GRB_DoubleAttr_RHS, 0);
          }
          // Turning off the self-loop constraint for new_val
          if (constrs_.find({var.name(), new_val}) != constrs_.end()) {
            model.getConstrByName(constrName(var, new_val)).set(GRB_DoubleAttr_RHS, big_m_);
          }
        }
      }
      cur_constrs_for = s;
    }


   private:
    double bigMConstantFromDeadendPenalty(HackedPrSasProblem const& sas_problem) {
      double min_positive_cost = gpt::dead_end_value.double_value();
      for (PrSasAction const& a : sas_problem.actions()) {
        // TODO: Is this still OK when we use the cost constrained OMC varying
        // each constraint as objective function?
        double cost_a = a.costVector()[ACTION_COST].double_value();
        if (cost_a > 0 && cost_a < min_positive_cost)
          min_positive_cost = cost_a;
      }
      return gpt::dead_end_value.double_value() / min_positive_cost;
    }

    std::string constrName(SasVariable const& var, SasVarValue const& v) const {
      return "selfloop_" + var.name() + "=" + to_string(v);
    }

    void addSelfLoopConstrForProjection(SasVariable const& var,
        SasVariableProjectedSSP const& proj_ssp,
        HashValEnumActionPtrToGRBVar proj_om,
        GRBModel& model,
        SasVarValue const& v0)
    {
      using HashValueToLinExpr = std::unordered_map<SasVarValue, GRBLinExpr>;
      HashValueToLinExpr non_self_in;
      HashValueToLinExpr self_in;

      auto const& domain = var.domain();
      std::vector<bool> has_self_in(domain.size());

      for (size_t iv = 0; iv < domain.size(); ++iv) {
        SasVarValue const& v = domain[iv];
        for (EnumProjectedAction const& a : proj_ssp.applicableActions(v)) {
          SasVarValuePr pr = a.eff();
          if (pr[v] == 1.0) {
            // P(v|v,a) == 1.0
            has_self_in[iv] = true;
            self_in[v] += proj_om[{v,&a}];
          }
          else {
            for (auto const& it : pr) {
              if (it.first == v) continue;
              // P(v'|v,a) > 0 and v' != v
              non_self_in[it.first] += it.second * proj_om[{v,&a}];
            }
          }
        }
      }

      for (size_t iv = 0; iv < domain.size(); ++iv) {
        if (!has_self_in[iv]) continue;
        SasVarValue const& v = domain[iv];
        double rhs = 0.0;
        if (v == v0) {
          // Turning off the self-loop constraint for v0 since it has an
          // "implicit" non-self-in flow of 1 by definition
          rhs = big_m_;
        }
        // Adding constraint:
        //    non_self_in > 0 implies self_in >= 0
        // translation:
        //    self_in <= big_M * non_self_in
        model.addConstr(self_in[v] - big_m_ * non_self_in[v], GRB_LESS_EQUAL, rhs,
                        constrName(var,v));
        constrs_.insert({var.name(),v});
      }
    }

    double big_m_;
    HashsetPairVarnameVal constrs_;
    state_t cur_constrs_for;
  };

  class CostConstraint {
   public:
    static std::string name() { return "single-c-omc-simple"; }

    void addExtraConstraints(state_t const&,
                             HackedPrSasProblem const& sas_problem,
                             std::vector<SasVariableProjectedSSPptr> const& proj_ssps,
                             VectorOfProjOM const& proj_om,
                             GRBModel& model)
    {
      ConstraintMap const& cost_constraints = sas_problem.constrs();
      size_t const n_constraints = cost_constraints.size();

      // LinExpr[i] will represent sum_{a in A} x_{V=v,a} C_i(a) for V equal to
      // the first Sas variable (any other could be used).
      std::vector<GRBLinExpr> cost_constr_lhs(n_constraints);
      for (PrSasAction const& a : sas_problem.actions()) {
        VecRationals const cost_vec_a = a.costVector();
        GRBLinExpr x_a = proj_ssps[0]->actionOM(a, model);
        for (size_t ci = 0; ci < n_constraints; ++ci) {
          cost_constr_lhs[ci] += cost_vec_a[cost_constraints.costIdx(ci)].double_value()
                                 * x_a;
        }
      }
      // Adding the secondary costs from the first projection (due to the tying
      // constraints, only one projection is necessary) to the cost constraints
      for (size_t i = 0; i < n_constraints; ++i) {
        model.addConstr(cost_constr_lhs[i], GRB_LESS_EQUAL,
                                          cost_constraints.maxExpectedValue(i),
                                          "cost_" + std::to_string(i));
      }
    }

    void updateExtraConstraints(state_t const&,
                                HackedPrSasProblem const&,
                                std::vector<SasVariableProjectedSSPptr> const&,
                                VectorOfProjOM const&,
                                GRBModel& model)
    { }
  };
};


template<typename ConstrGenerator>
class OccupationMeasureCountTemplate : public FactoredHeuristic {
 public:
  OccupationMeasureCountTemplate(problem_t const& problem, bool use_deadend_transformation,
                                 size_t cost_idx_for_obj_func = ACTION_COST)
    : FactoredHeuristic(ConstrGenerator::name(), problem),
      cond_sas_problem_sptr_(nullptr),
      cost_idx_for_obj_func_(cost_idx_for_obj_func)
  {
    // TODO: HACK: XXX: FIXME: improve the comparison between problem_t
    if (gpt::problem == &problem
        && gpt::cached_cond_sas_problem
        && gpt::cached_cond_sas_problem->hasBeenDeadEndTransformed() == use_deadend_transformation)
    {
      std::cout << "[" << name() << "] Using cached HackedPrSasProblem\n";
      cond_sas_problem_sptr_ = gpt::cached_cond_sas_problem;
    }
    else {
      cond_sas_problem_sptr_.reset(new HackedPrSasProblem(problem, use_deadend_transformation));
      if (!gpt::cached_cond_sas_problem)
        gpt::cached_cond_sas_problem = cond_sas_problem_sptr_;
    }
  }

  ~OccupationMeasureCountTemplate() {
    std::cout << "[" << name() << "] GRB num. variables = "
              << model_->get(GRB_IntAttr_NumVars) << std::endl
              << "[" << name() << "] GRB num. constraints = "
              << model_->get(GRB_IntAttr_NumConstrs) << std::endl;
  }


 protected:
  double computeValue(state_t const& s) {
    if (!model_) {
      model_ = Gurobi::newModel();
      buildModelFor(s);
    }
    else {
      changeModelFor(s);
    }
    double value = solveModel();
    assert(value >= 0);
    assert(value <= gpt::dead_end_value.double_value());
    if (gpt::print_lp) {
      static size_t i = 0;
      std::cout << "i = " << i << std::endl;
      model_->write(name() + "." + to_string(i) + ".lp");
      i++;
    }
//    printGurobiSolution();
//    std::cout << "h-" << name() << "(" << s.toStringFull(gpt::problem) << ") = "
//              << value << std::endl;
    return value;
  }

 private:
  void buildModelFor(state_t const& s);


  bool checkFlowConstrRHSofProjSSP(size_t proj_idx, SasVarValue const& v, double rhs) {
    model_->update();
    SasVariable const& var = cond_sas_problem_sptr_->variables()[proj_idx];
    std::string constr_name("flow_" + var.name() + "_" + std::to_string(v));
    return model_->getConstrByName(constr_name).get(GRB_DoubleAttr_RHS) == rhs;
  }

  void changeModelFor(state_t const& s) {
    auto const& sas_variables = cond_sas_problem_sptr_->variables();
    for (size_t iv = 0; iv < sas_variables.size(); ++iv) {
      SasVariable const& var = sas_variables[iv];
      assert(var.valueAt(cur_model_for_) == var.valueAt(s)
             || checkFlowConstrRHSofProjSSP(iv, var.valueAt(cur_model_for_), 1.0));
      assert(var.valueAt(cur_model_for_) == var.valueAt(s)
             || checkFlowConstrRHSofProjSSP(iv, var.valueAt(s), 0.0));

      proj_ssps_[iv]->changeInitialState(var.valueAt(s), *model_);
      assert(var.valueAt(cur_model_for_) == var.valueAt(s)
             || checkFlowConstrRHSofProjSSP(iv, var.valueAt(cur_model_for_), 0.0));
      assert(var.valueAt(cur_model_for_) == var.valueAt(s)
             || checkFlowConstrRHSofProjSSP(iv, var.valueAt(s), 1.0));
    }
    extra_constr_generator_.updateExtraConstraints(s, *cond_sas_problem_sptr_, proj_ssps_,
                                                   proj_occ_measure_, *model_);
    cur_model_for_ = s;
  }


  double solveModel() {
    try {
      model_->optimize();
      int gurobi_status = model_->get(GRB_IntAttr_Status);
      if (gurobi_status == GRB_INFEASIBLE) {
        // For this particular problem, deterministic operator count, an
        // infeasible problem means that the current state is a dead-end.
        return gpt::dead_end_value.double_value();
      }
      else if (gurobi_status != GRB_OPTIMAL) {
        std::cout << "[lp-dual]: Gurobi DID NOT find the optimal solution. "
                  << "GRB_IntAttr_Status = " << gurobi_status << std::endl
                  << "Gurobi's error desc = '"
                  << Gurobi::errorCodeTranslation(gurobi_status)
                  << "'" << std::endl << "Quitting" << std::endl;
        exit(181);
      }
      double solution = model_->get(GRB_DoubleAttr_ObjVal);
      return solution;
    }
    catch (GRBException const& e) {
      std::cout << "Gr Exception caught while optimizing. Error code = "
                << e.getErrorCode()
                << std::endl
                << e.getMessage() << std::endl
                << "For now quitting..." << std::endl;
      exit(220);
    }
    return -1;
  }

  void printGurobiSolution() const;

  std::shared_ptr<HackedPrSasProblem> cond_sas_problem_sptr_;
  size_t cost_idx_for_obj_func_;

  std::vector<SasVariableProjectedSSPptr> proj_ssps_;
  VectorOfProjOM proj_occ_measure_;
  ConstrGenerator extra_constr_generator_;
  state_t cur_model_for_;

  GRBModelSPtr model_;
};

using SimpleOccupationMeasureCount = OccupationMeasureCountTemplate<
                                OccMeasCountConstraintsGenerator::NoExtraConstraint>;
using OccupationMeasureCountSelfLoopConstr = OccupationMeasureCountTemplate<
                                OccMeasCountConstraintsGenerator::SelfLoopConstr>;
using CostConstrOMCSimple = OccupationMeasureCountTemplate<
                                OccMeasCountConstraintsGenerator::CostConstraint>;

#endif  // USE_GUROBI
#endif  // HEURISTICS_MC_H
