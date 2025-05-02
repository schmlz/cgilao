#ifndef HEURISTICS_OP_COUNT_H
#define HEURISTICS_OP_COUNT_H

#include <iostream>
#include <unordered_map>
#include <vector>
#include <set>

#include "../utils/lp_solver_wrapper.h"
using namespace LPWrappers;
#if defined USE_GUROBI
using LPWrap = LPWrapper<LPSolver::GUROBI>;
#elif defined USE_CPLEX
using LPWrap = LPWrapper<LPSolver::CPLEX>;
#endif

#include "heuristic_iface.h"
#include "../representations/pr_sas_iface.h"

// NOTE: In order to consider this a proper partition, all the missing operators should
// be considered in an extra category 'other' that is ignored for the operator counting
// heuristic
struct OperatorPartition {
  // Right now, we only need is a quick way to get the gurobi variable related to the
  // action in question, so storing the index of the variable is good enough
  std::vector<LPWrap::Var*> always_produce;
  std::vector<LPWrap::Var*> always_consume;
  std::vector<LPWrap::Var*> sometimes_produce;
  std::vector<LPWrap::Var*> sometimes_consume;
};


using MapValToOpPartition = std::unordered_map<SasVarValue, OperatorPartition>;
using SasValuationPtrToGRBVar = std::unordered_map<SasValuation const*, LPWrap::Var>;

namespace OpCountConstraintsGenerator {
  class NoExtraConstraint {
   public:
    static std::string name() { return "OpCount"; }
    void addExtraConstraints(state_t const&,
                             NonConditionalPrSasProblem const&,
                             LPWrap&,
                             SasValuationPtrToGRBVar const&)
    { }
    void updateExtraConstraints(state_t const&,
                             NonConditionalPrSasProblem const&,
                             LPWrap&,
                             SasValuationPtrToGRBVar const&)
    { }
  };

  class RegroupConstraint {
   public:
    static std::string name() { return "ROC"; }
    void addExtraConstraints(state_t const&,
                             NonConditionalPrSasProblem const& sas_problem,
                             LPWrap& model,
                             SasValuationPtrToGRBVar const& eff_to_grbvar)
    {
      auto const& actions = sas_problem.actions();
      for (size_t ai = 0; ai < actions.size(); ++ai) {
        NonConditionalPrSasAction const& a = actions[ai];
        for (size_t ei = 0; ei < a.size(); ++ei) {
          double p_e_i = a.pr(ei).double_value();
          SasValuation const& eff_i = a.eff(ei);
          auto const it = eff_to_grbvar.find(&eff_i);
          assert(it != eff_to_grbvar.end());
          LPWrap::Var const& y_e_i = it->second;
          for (size_t ej = ei+1; ej < a.size(); ++ej) {
            double p_e_j = a.pr(ej).double_value();
            SasValuation const& eff_j = a.eff(ej);
            auto const jt = eff_to_grbvar.find(&eff_j);
            assert(jt != eff_to_grbvar.end());
            LPWrap::Var const& y_e_j = jt->second;
            // model.addConstr(p_e_i * y_e_j - p_e_j * y_e_i, GRB_EQUAL, 0.0);
            model.add_constraint(p_e_i * y_e_j - p_e_j * y_e_i == 0.0);
          }
        }
        if (get_max_resident_mem_in_kb() > gpt::max_rss_kb) {
          EXIT("[ROC] addExtraConstraints used too much memory");
        }
      }
    }
    void updateExtraConstraints(state_t const&,
                             NonConditionalPrSasProblem const&,
                             LPWrap&,
                             SasValuationPtrToGRBVar const&)
    { }
  };

  class CostConstraint {
   public:
    static std::string name() { return "CostConstr"; }
    void addExtraConstraints(state_t const&,
                             NonConditionalPrSasProblem const& sas_problem,
                             LPWrap& model,
                             SasValuationPtrToGRBVar const& eff_to_grbvar)
    {
      ConstraintMap const& cost_constraints = sas_problem.constrs();
      size_t const n_constraints = cost_constraints.size();

      // LinExpr[i] will represent sum_{a in A, e \in eff(a)} Y_{a,e}C_i(a)
      std::vector<LPWrap::Row> cost_constr_lhs(n_constraints);
      for (NonConditionalPrSasAction const& a : sas_problem.actions()) {
        VecRationals const cost_vec_a = a.costVector();
        for (size_t ei = 0; ei < a.size(); ++ei) {
          SasValuation const& eff_i = a.eff(ei);
          auto const it = eff_to_grbvar.find(&eff_i);
          assert(it != eff_to_grbvar.end());
          LPWrap::Var const& y_a_ei = it->second;
          for (size_t ci = 0; ci < n_constraints; ++ci) {
            cost_constr_lhs[ci] += cost_vec_a[cost_constraints.costIdx(ci)].double_value()
                                   * y_a_ei;
          }
        }
      }
      // Adding the secondary costs from the first projection (due to the tying
      // constraints, only one projection is necessary) to the cost constraints
      for (size_t i = 0; i < n_constraints; ++i) {
        // model.addConstr(cost_constr_lhs[i], GRB_LESS_EQUAL,
        //                                   cost_constraints.maxExpectedValue(i),
        //                                   "cost_" + std::to_string(i));
        model.add_constraint(cost_constr_lhs[i] - cost_constraints.maxExpectedValue(i) <= 0,
                                          "cost_" + std::to_string(i));
      }
    }
    void updateExtraConstraints(state_t const&,
                             NonConditionalPrSasProblem const&,
                             LPWrap&,
                             SasValuationPtrToGRBVar const&)
    { }
  };

  class CostConstraintRegrouped {
   public:
    static std::string name() { return "single-c-roc"; }
    void addExtraConstraints(state_t const& s,
                             NonConditionalPrSasProblem const& sas_problem,
                             LPWrap& model,
                             SasValuationPtrToGRBVar const& eff_to_grbvar)
    {
      regroup_constr_.addExtraConstraints(s, sas_problem, model, eff_to_grbvar);
      cost_constr_.addExtraConstraints(s, sas_problem, model, eff_to_grbvar);
    }
    void updateExtraConstraints(state_t const& s,
                             NonConditionalPrSasProblem const& sas_problem,
                             LPWrap& model,
                             SasValuationPtrToGRBVar const& eff_to_grbvar)
    {
      regroup_constr_.updateExtraConstraints(s, sas_problem, model, eff_to_grbvar);
      cost_constr_.updateExtraConstraints(s, sas_problem, model, eff_to_grbvar);
    }
   private:
    RegroupConstraint regroup_constr_;
    CostConstraint cost_constr_;
  };
};

template<typename ConstrGenerator>
class OperatorCountTemplate : public FactoredHeuristic {
 public:
  OperatorCountTemplate(problem_t const& problem, bool use_deadend_transformation,
                        size_t cost_idx_for_obj_func = ACTION_COST)
  : FactoredHeuristic(ConstrGenerator::name(), problem),
    cond_sas_problem_sptr_(nullptr), sas_problem_sptr_(nullptr), model_(nullptr),
    cost_idx_for_obj_func_(cost_idx_for_obj_func)
  {
    // TODO: HACK: XXX: FIXME: improve the comparison between problem_t
    if (gpt::problem == &problem
        && gpt::cached_cond_sas_problem
        && gpt::cached_cond_sas_problem->hasBeenDeadEndTransformed() == false)
    {
      std::cout << "[" << name() << "] Using cached HackedPrSasProblem\n";
      cond_sas_problem_sptr_ = gpt::cached_cond_sas_problem;
    }
    else {
      cond_sas_problem_sptr_.reset(new HackedPrSasProblem(problem, false));
      if (!gpt::cached_cond_sas_problem)
        gpt::cached_cond_sas_problem = cond_sas_problem_sptr_;
    }

    if (gpt::cached_cond_sas_problem == cond_sas_problem_sptr_
        && gpt::cached_non_cond_sas_problem
        && gpt::cached_cond_sas_problem->hasBeenDeadEndTransformed() == use_deadend_transformation)
    {
      std::cout << "[" << name() << "] Using cached NonConditionalPrSasProblem\n";
      sas_problem_sptr_ = gpt::cached_non_cond_sas_problem;
    }
    else {
      sas_problem_sptr_.reset(new NonConditionalPrSasProblem(*cond_sas_problem_sptr_,
                                                             use_deadend_transformation));
      if (!gpt::cached_non_cond_sas_problem)
        gpt::cached_non_cond_sas_problem = sas_problem_sptr_;
    }

    assert(cost_idx_for_obj_func_ < sas_problem_sptr_->actions()[0].costVector().size());
  }

  ~OperatorCountTemplate() {
    // Causing issues because the environmnent is being freed before the model
    // std::cout << "[" << name() << "] GRB num. variables = "
    //           << model_->get(GRB_IntAttr_NumVars) << std::endl
    //           << "[" << name() << "] GRB num. constraints = "
    //           << model_->get(GRB_IntAttr_NumConstrs) << std::endl;
  }


 protected:
  double computeValue(state_t const& s) {
    if (!model_) {
      // model_ = Gurobi::newModel();
      model_.reset(new LPWrap);
      buildModelFor(s);
      extra_constr_generator_.addExtraConstraints(s, *sas_problem_sptr_, *model_,
                                                  eff_to_grbvar_);
    }
    else {
      changeModelFor(s);
      extra_constr_generator_.updateExtraConstraints(s, *sas_problem_sptr_, *model_,
                                                     eff_to_grbvar_);
    }
    model_->update();
    double value = solveModel();
    assert(value >= 0);
    assert(value <= gpt::dead_end_value.double_value() + gpt::epsilon);
//    std::cout << "h-" << name() << "(" << s.toStringFull(gpt::problem)
//              << ") = " << value << std::endl;
    return value;
  }


 private:
  std::pair<int,int> possibleNetChangeFrom(state_t const& s,
      SasVariable const& variable, SasVarValue const& value) const;

  void buildOperatorPartition();
  void buildModelFor(state_t const& s);
  void changeModelFor(state_t const& s);
  void changeConstraintFor(state_t const& s, SasVariable const& variable,
                           SasVarValue const& val);

  double solveModel();

  std::shared_ptr<HackedPrSasProblem> cond_sas_problem_sptr_;
  std::shared_ptr<NonConditionalPrSasProblem> sas_problem_sptr_;
  std::shared_ptr<LPWrap> model_; // TODO(jsch) fix pointer (used to be shared_ptr to gurobi model)
  size_t cost_idx_for_obj_func_;

  // Keep track of constraints that may need to be updated at some stage
  //
  // NOTE: this used to be done with gurobi's getConstrByName, but cplex doesn't seem to have an
  //       equivalent
  std::unordered_map<std::string, LPWrap::Constr> mutable_constraints_;

  MapValToOpPartition op_partition_;
  SasValuationPtrToGRBVar eff_to_grbvar_;

  // This variable represents the states (in the SAS+ representation) for which
  // the current model was built. When the model is changed, this variable is
  // used for finding only the necessary changes and them update to the new
  // state that represented by the model.
  SasValuation model_built_for_;

  ConstrGenerator extra_constr_generator_;

#ifndef NDEBUG
  std::set<std::string> used_constr_names_;
#endif
};

using OperatorCount = OperatorCountTemplate<
                                OpCountConstraintsGenerator::NoExtraConstraint>;
using RegroupedOperatorCount = OperatorCountTemplate<
                                OpCountConstraintsGenerator::RegroupConstraint>;
using CostConstrROC = OperatorCountTemplate<
                                OpCountConstraintsGenerator::CostConstraintRegrouped>;

#endif  // HEURISTICS_OP_COUNT_H