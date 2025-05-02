#ifndef REPRESENTATION_PROJECTED_SSP_H
#define REPRESENTATION_PROJECTED_SSP_H

#include <iostream>
#include <cassert>
#include <unordered_map>

#include <boost/functional/hash.hpp>

#include "pr_sas_iface.h"

#if defined USE_GUROBI

#include "gurobi_c++.h"  // In the include path

// TODO(fwt): Use ProbDistIface and probably improve ProbDistIface
using SasVarValuePr = std::unordered_map<SasVarValue, double>;

class EnumProjectedAction {
 public:
  EnumProjectedAction(std::string const& name, PrSasAction const& original_action,
                      SasVarValuePr eff)
    : name_(name),
      original_action_(original_action),
      eff_(eff)
  { }

  ~EnumProjectedAction() { }

  std::string const& name() const { return name_; }
  VecRationals const& costVector() const { return original_action_.costVector(); }
  PrSasAction const& originalAction() const { return original_action_; }

  SasVarValuePr const& eff() const { return eff_; }

 private:
  std::string name_;
  PrSasAction const& original_action_;
  SasVarValuePr eff_;
};

namespace std {
  template<> struct hash<EnumProjectedAction> {
    size_t operator()(EnumProjectedAction const& a) const {
      return hash<string>()(a.name());
    }
  };
}

using PairValEnumActionPtr = std::pair<SasVarValue, EnumProjectedAction const*>;
using HashValEnumActionPtrToGRBVar = std::unordered_map<PairValEnumActionPtr, GRBVar,
                                                      boost::hash<PairValEnumActionPtr>>;


// FWT: cannot inherit from ConstrSSPIface because it uses state_t as state
// instead of a generic class that we could tie with SasVarValue.
//
// Still the methods should be the same so that we can templatize everything
// else that is shared between SSPIface, ConstrSSPIface, and
// SasVariableProjectedSSP
//
class SasVariableProjectedSSP {
 public:
  SasVariableProjectedSSP(HackedPrSasProblem const& sas_problem,
                          SasVariable const& proj_var)
    : sas_problem_(sas_problem), proj_var_(proj_var)
  {
    assert(sas_problem_.initial_state().find(proj_var_)
                                         != sas_problem_.initial_state().end());
    initial_state_ = sas_problem_.initial_state().find(proj_var_)->second;

    SasVarValue max_val = 0;
    for (SasVarValue const& v : proj_var_.domain()) {
      if (v > max_val) max_val = v;
    }
    artificial_goal_ = max_val + 1;

    /*
     * Building the artificial action a_g
     */
    sas_artificial_action_.reset(new PrSasAction());
    sas_artificial_action_->name() = "a_{g for " + proj_var_.name() + "}";
    // FWT: Maybe the cost of a_g should be the terminal cost. For now it's the
    // zero vector
    size_t size_cost_vec = sas_problem.actions()[0].costVector().size();
    sas_artificial_action_->costVector() = VecRationals(size_cost_vec, Rational(0));
    sas_artificial_action_->setOriginalAction(nullptr);
    SasValuation a_g_eff;
    a_g_eff[proj_var_] = artificial_goal_;
    sas_artificial_action_->pushEffect(1, a_g_eff);

    auto const goal_it = sas_problem_.goal().find(proj_var_);
    if (goal_it != sas_problem_.goal().end()) {
      // In the goal, proj_var_ is defined
      sas_artificial_action_->prec()[proj_var_] = goal_it->second;
    }

    buildProjectedActionVector();

    /*
     * Building the grounded version of the sas_artificial_action_
     */
    SasVarValuePr det_goal;
    det_goal[artificial_goal_] = 1.0;
    enum_artificial_goal_.reset(new EnumProjectedAction(
                                               sas_artificial_action_->name(),
                                               *sas_artificial_action_,
                                               det_goal));
    if (goal_it != sas_problem_.goal().end()) {
      // In the goal, proj_var_ is defined
      actions_[goal_it->second].push_back(*enum_artificial_goal_);
    }
    else {
      // Goal action is applicable everywhere because the goal is not defined
      // for this action
      for (SasVarValue const& v : proj_var_.domain()) {
        actions_[v].push_back(*enum_artificial_goal_);
      }
    }
  }

  SasVariableProjectedSSP(HackedPrSasProblem const& sas_problem,
                          SasVariable const& proj_var,
                          SasVarValue const& initial_state)
    : SasVariableProjectedSSP(sas_problem, proj_var)
  {
    assert(proj_var_.isValidValue(initial_state));
    changeInitialState(initial_state);
  }


  // All these are deleted to avoid implementing a very nasty deep copy since a
  // lot of things carry pointers/references to each other.
  SasVariableProjectedSSP(SasVariableProjectedSSP const& other) = delete;
  SasVariableProjectedSSP& operator=(SasVariableProjectedSSP& other) = delete;


  ~SasVariableProjectedSSP() { } //std::cout << "DTOR for " << this << std::endl; }


  std::string name() const {
    return "Proj(" + sas_problem_.name() + "," + proj_var_.name() + ")";
  }

  // Returns s0, the initial state
  SasVarValue const& s0() const {
    assert(proj_var_.isValidValue(initial_state_));
    return initial_state_;
  }

  SasVarValue const& artificialGoal() const { return artificial_goal_;  }


  // Returns true if s \in G
  bool isGoal(SasVarValue const& v) const {
    assert(v == artificial_goal_ || proj_var_.isValidValue(v));
    return v == artificial_goal_;
  }

  // Returns true if |A(s)| > 0
//  bool hasApplicableActions(SasVarValue const& v) const {
//    assert(v == artificial_goal_ || proj_var_.isValidValue(v));
//    if (v == artificial_goal_) return false;
//    if (artificial_action_.prec().size() == 0) return true;
//    assert(artificial_action_.prec().size() == 1);
//    if (artificial_action_.prec().begin()->second == v) return true;
//    for (PrSasAction const& a : sas_problem_.actions()) {
//      if (isApplicable(v, a)) return true;
//    }
//    return false;
//  }

  std::deque<EnumProjectedAction> const& applicableActions(SasVarValue const& v) const
  {
    static const std::deque<EnumProjectedAction> empty;
    auto const it = actions_.find(v);
    if (it != actions_.end())
      return it->second;
    return empty;
  }

  // Returns true if a in A(s)
  bool isApplicable(SasVarValue const& v, PrSasAction const& a) const {
    assert(v == artificial_goal_ || proj_var_.isValidValue(v));
    auto const it = a.prec().find(proj_var_);
    return it == a.prec().end() || it->second == v;
  }



  // Returns the main cost, i.e., the cost being optimized. We assume that this
  // cost is first cost in the costVector.
  VecRationals costVector(PrSasAction const& a) const {
    return a.costVector();
  }

  // Return the mapping from k (cost index) to c (constant) representing the
  // constraint: E_\pi[C_k] <= c
  ConstraintMap const& constraints() const { return sas_problem_.constrs(); }

  // If cost_idx_for_obj_func < 0, then no variable is added to the
  // objective function (i.e., all the coefficients for the newly created
  // variables will be 0). Otherwise, the cost_idx_for_obj_func position of
  // the costVector will be used as coefficient in the objective function.
  HashValEnumActionPtrToGRBVar buildDualFlowConstraints(GRBModel& model,
      int cost_idx_for_obj_func, bool add_sink_variable = false) const;

  void changeInitialState(SasVarValue const& v) {
    assert(proj_var_.isValidValue(v));
    initial_state_ = v;
  }

  void changeInitialState(SasVarValue const& v, GRBModel& model) {
    SasVarValue old_v0 = s0();
    if (old_v0 != v) {
      // Internal changes
      changeInitialState(v);
      // Remove the injected mass from old_v0 in the LP model
      setFlowConstrTo(old_v0, 0, model);
      // inject mass of 1 in v
      setFlowConstrTo(v, 1, model);
    }
  }

  SasVarDomain const& varOriginalDomain() const {
    return proj_var_.domain();
  }

  GRBConstr getFlowConstrFor(SasVarValue const& v, GRBModel& model) {
    assert(isFlowConstrNameDefined(v, model));
    return model.getConstrByName(flowConstrName(v));
  }

  // Returns the GRBLinExpr representing sum_{v} x_{v,a}
  // TODO: EFFICIENCY: Maybe build cache PrSasAction -> Container(EnumProjectedAction*)
  GRBLinExpr actionOM(PrSasAction const& a, GRBModel& model) const {
#if not defined NDEBUG
    bool is_empty = true;
#endif
    GRBLinExpr x_a;
    for (auto const& pair : actions_) {
      SasVarValue const& v = pair.first;
      for (EnumProjectedAction const& enum_a : pair.second) {
        if (enum_a.originalAction() == a) {
          // TODO: EFFICIENCY: FIXME: Decide if it's better to keep a hash of the
          // variables or if it's better to query the model as it is now.
          x_a += model.getVarByName(omName(v,enum_a));
#if not defined NDEBUG
          is_empty = false;
#endif
        }
      }
    }
    assert(!is_empty);
    return x_a;
  }

  void dump() const;

 private:
  std::string flowConstrName(SasVarValue const& v) const {
    return "flow_" + proj_var_.name() + "_" + to_string(v);
  }

  std::string omName(SasVarValue const& v, EnumProjectedAction const& a) const {
    return "x_{" + proj_var_.name() + "=" + to_string(v) + "," + a.name() + "}";
  }

  void setFlowConstrTo(SasVarValue const& v, double rhs, GRBModel& model) const {
    model.getConstrByName(flowConstrName(v)).set(GRB_DoubleAttr_RHS, rhs);
  }

  // DEBUG method
  bool isFlowConstrNameDefined(SasVarValue const& v, GRBModel& model) const;


  /*
   * GUARANTEE: expand WILL clear the pr before populating it.
   */
  void expand(PrSasAction const& a, SasVarValue const& v, std::vector<SasVarValuePr>& v_pr) const {
    assert(v != artificial_goal_);
    assert(proj_var_.isValidValue(v));

    v_pr.clear();
    SasVarValuePr aux;
    v_pr.push_back(aux);

    assert(isApplicable(v, a));

    // TODO(fwt): improve the detection of the artificial_action_
    if (a.name() == sas_artificial_action_->name()) {
      v_pr[0][artificial_goal_] = 1;
      return;
    }

    /*
     * Experiment 2016.10.07-Testing_different_implementations_of_h-omc-sl-constr
     * (performed on commit fc416178b703f7c556c62f9dfffb1e88959747fb) showed
     * that normalization is necessary to avoid numerical instability for
     * omc-sl-constr.
     *
     * TODO(rational numbers): once probabilities are rational numbers again,
     * then normalization HERE can be discarded since it's only summing
     * probabilities.
     */
    double total = 0.0;
    for (size_t ei = 0; ei < a.size(); ++ei) {
      VectorSasEffect v_eff = a.eff(ei);
      double const pr_eff = a.pr(ei).double_value();
      total += pr_eff;
      bool v_eff_affects_proj_var = false;
      for (SasEffect const& c_eff : v_eff) {
        if (c_eff.eff().var == proj_var_) {
          v_eff_affects_proj_var = true;
          SasVarValue const& new_v = c_eff.eff().value;
          auto const cond_it = c_eff.cond().find(proj_var_);
          if (c_eff.isConditional() && cond_it == c_eff.cond().end() && new_v != v)
          {
            // This effect is conditional on a set of variables that doesn't
            // contain proj_var and it changes the value to proj_var to
            // something different to v (the current value). Thus, we need to
            // consider 2 cases: when the condition is true (and the value
            // changes) and the condition is false (and the value stays the
            // same, i.e., self-loop).
            size_t const initial_size = v_pr.size();
            // FWT: Currently, this has undefined behavior since the first
            // last are iterators of v_pr itself.
            // v_pr.insert(v_pr.end(), v_pr.begin(), v_pr.end());
            for (size_t i = 0; i < initial_size; ++i) {
              // Duplicating entry i
              v_pr.push_back(v_pr[i]);
              // Condition is false and we self-loop
              v_pr[i][v] += pr_eff;
              // Condition is true and we change value
              assert(v != new_v);
              v_pr[initial_size + i][new_v] += pr_eff;
            }
          }
          else {
            // This effect is equivalent to a simple change from v to the new value
            for (auto& pr : v_pr) { pr[new_v] += pr_eff; }
          }
          // Only one conditional deterministic effect in a VectorSasEffect can
          // affect any given variable
          break;
        }
      }
      if (!v_eff_affects_proj_var) {
        // self-loop
        for (auto& pr : v_pr) { pr[v] += pr_eff; }
      }
    }
    assert(total > 0);
    assert(total < 1.001);

    // Normalizing
    for (auto& pr : v_pr) {
      for (auto& it : pr) {
        it.second = it.second / total;
      }
    }
  }

  void buildProjectedActionVector();


  HackedPrSasProblem const& sas_problem_;
  SasVariable const& proj_var_;
  SasVarValue initial_state_;
  SasVarValue artificial_goal_;
  std::shared_ptr<PrSasAction> sas_artificial_action_;
  std::shared_ptr<EnumProjectedAction> enum_artificial_goal_;
  std::unordered_map<SasVarValue, std::deque<EnumProjectedAction>> actions_;
};

using SasVariableProjectedSSPptr = std::shared_ptr<SasVariableProjectedSSP>;

#endif  // USE_GUROBI
#endif  // REPRESENTATION_PROJECTED_SSP_H
