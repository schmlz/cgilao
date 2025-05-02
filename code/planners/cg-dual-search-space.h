#ifndef PLANNER_CG_DUAL_SEARCH_SPACE_H
#define PLANNER_CG_DUAL_SEARCH_SPACE_H

#include <iostream>
#include <unordered_map>

#include "../ssps/ssp_iface.h"
#include "../heuristics/heuristic_cssp_factory.h"
#include "../ssps/policy.h"
#include "../utils/gurobi.h"


struct StateActionPtr {
  state_t state;
  action_t const* action_ptr;
};

// Hash for StateAction. This is based on boost::hash_combine
// http://www.boost.org/doc/libs/1_65_1/doc/html/hash/reference.html
namespace std {
  template<> struct hash<StateActionPtr> {
    size_t operator()(StateActionPtr const& s_a) const {
      size_t key = std::hash<state_t>()(s_a.state);
      size_t action_ptr_key = std::hash<std::string>()(s_a.action_ptr ?
                                                          s_a.action_ptr->name() : "");
      key ^= action_ptr_key + 0x9e3779b9 + (key << 6) + (key >> 2);
      return key;
    }
  };
  template<> struct equal_to<StateActionPtr> {
    bool operator()(StateActionPtr const& x, StateActionPtr const& y) const {
      return (x.action_ptr == y.action_ptr) && (x.state == y.state);
    }
  };
}

using SetOfColumns = std::unordered_set<StateActionPtr>;
using SetStates = std::unordered_set<state_t, hashState>;


#if defined USE_GUROBI

/*
 * Private class to wrap all the search space variables and helper methods
 */
class SearchSpace {
 public:
  SearchSpace(ConstrSSPIface const& cssp, CachedVectorHeuristicWrapper& cached_h_vector,
              action_t const& a_fringe_or_deadend, bool use_instability_sink)
    : cssp_(cssp), cached_h_vector_(cached_h_vector), a_fringe_or_deadend_(&a_fringe_or_deadend),
      model_status_(ModelStatus::NOT_OPTIMIZED)
  {
    try {
      model_ = Gurobi::newModel();
      std::cout << "[CG-DUAL] Forcing the usage of simplex and turning on FarkasDualProof\n";
      model_->getEnv().set(GRB_IntParam_Method, 0);
      model_->getEnv().set(GRB_IntParam_InfUnbdInfo, 1);
    }
    catch (GRBException const& e) {
      std::cout << "GRBException caught. Error code = " << e.getErrorCode()
                << std::endl
                << e.getMessage() << std::endl
                << "For now quitting..." << std::endl;
      EXIT("GRBException caught");
    }
    buildBaseCase(use_instability_sink);
  }

  ~SearchSpace() { }

  SetOfColumns const& columnsAvailable() const { return columns_available_; }

  bool solveRMP() {
    assert(model_status_ == ModelStatus::NOT_OPTIMIZED);
    uint64_t remaining_time_in_usec = 0;
    if (gpt::_deadline_ && gpt::_deadline_->remainingTimeInUsec(remaining_time_in_usec)) {
      model_->getEnv().set(GRB_DoubleParam_TimeLimit,
                            remaining_time_in_usec / (double) 1000000.0);
    }
    model_->optimize();

    int gurobi_status = model_->get(GRB_IntAttr_Status);
    switch (gurobi_status) {
     case GRB_OPTIMAL:
      model_status_ = ModelStatus::OPTIMAL;
      return true;

     case GRB_INFEASIBLE:
      model_status_ = ModelStatus::INFEASIBLE;
      return false;

     case GRB_TIME_LIMIT:
      assert(gpt::_deadline_);
      assert(gpt::_deadline_->remainingTimeInUsec(remaining_time_in_usec));
      std::cout << "[CG-DUAL]: no more time to solve the problem."
                << " FWT: figure out what to do now." << std::endl
                << "For now quitting..." << std::endl;
      EXIT("Gurobi: Time limit reached");

     default:
      std::cout << "Unpredicted gurobi model status. "
              << "GRB_IntAttr_Status = " << gurobi_status << std::endl
              << "Gurobi's error desc = '"
              << Gurobi::errorCodeTranslation(gurobi_status)
              << "'" << std::endl << "Quitting" << std::endl;
      EXIT("Unpredicted gurobi model status");
    }
    assert(false);
    return false;
  }

  double optimalValueOfRMP() const {
    assert(model_status_ == ModelStatus::OPTIMAL);
    return model_->get(GRB_DoubleAttr_ObjVal);
  }

  ProbPolicy extractPolicyFromRMP() const;

  bool hasFlowConstrFor(state_t const& s) const {
    return flow_constr_name_.find(s) != flow_constr_name_.end();
  }

  HashsetState const& artificialGoals() const { return artificial_goals_; }

  HashsetState reachableArtificialGoals() const {
    assert(model_status_ == ModelStatus::OPTIMAL);
    HashsetState rv;
    for (state_t const& s : artificial_goals_) {
      if (isReachableArtificialGoal(s)) {
        rv.insert(s);
      }
    }
    return rv;
  }

  bool isInternal(state_t const& s) const {
    // XXX: TRIPLE CHECK LOGIC of methods using it because original goals and dead-ends are internal
    // states!!
    return internal_states_.find(s) != internal_states_.end();
  }

  bool isArtificialGoal(state_t const& s) const {
    return artificial_goals_.find(s) != artificial_goals_.end();
  }

  bool isReachableArtificialGoal(state_t const& s) const {
    assert(model_status_ == ModelStatus::OPTIMAL);
#if 0
    // This is the i-dual approach: all the mass that reaches a state that is too small is ignored
    // because it could be due to numerical instability. However, this makes the following feasible
    // problem (with Pr(goal) = 1) INFEASIBLE!!
    /*
      solver_cssp -r 1507760088 --no_instability_sink --grb-method 0 -p cg-dual -h h-vec-max -R 1 \
                  ppddls.git_ignore_me/exbw_extension1_domain-NO-COND.ppddl \
                  ppddls.git_ignore_me/exbw_p02-n3-N5-s2-constr_p-0.2.ppddl
    */
    static const double min_expected_value = model_->getEnv().get(GRB_DoubleParam_OptimalityTol);
#else
    // Allowing all the states with positive flow to be considered (see comment above)
    static const double min_expected_value = 0.0;
#endif
    if (!isArtificialGoal(s)) return false;
    // s is an artificial goal so a_fringe_or_deadend_ assumes the semantics of a_fringe
    auto it = occupancy_measure_.find({s, a_fringe_or_deadend_});
    assert(it != occupancy_measure_.end());
    GRBVar const& x_s_fringe = it->second;
    return x_s_fringe.get(GRB_DoubleAttr_X) > min_expected_value;
  }

  void addColumns(SetOfColumns const& columns) {
    assert(columns.size() > 0);
    // Since new columns will be added, the current solution will become invalid
    model_status_ = ModelStatus::NOT_OPTIMIZED;

    std::cout << "[CG-DUAL expansion] Columns to be added: ";
    for (auto const& c : columns) {
      std::cout << "(" << c.state.toStringFull(gpt::problem) << ", "
                << (c.action_ptr ? c.action_ptr->name() : "nullptr")
                << "), ";
    }
    std::cout << std::endl;

    // Finding the new states that are reachable when we add the given columns. This is necessary to
    // be able to build the "pre-conditions" (constraints and variables) necessary to add the new
    // columns
    SetStates new_states = newReachableStatesFrom(columns);
    // Adding the flow constraints constraint for each new state. These flow constraints are
    // initially empty
    for (state_t const& s : new_states) {
      addEmptyFlowConstrFor(s);
    }
    std::cout << "[CG-DUAL expansion] New set of reachable states: {";
    for (state_t const& s : new_states) {
      std::cout << s.toStringFull(gpt::problem) << ", ";
    }
    std::cout << "}\n";
    model_->update();

    // Adding the fringe columns, i.e., the columns (s,a_fringe) for all states s in new_states.
    // Note that there is no need to call a model update after it since we won't refer to these
    // variable until we are done.
    for (state_t const& s : new_states) {
      insertStateToSearchSpace(s);
    }


    // Now that we have all the necessary new constraints (and variables), we will add the
    // requested columns
    for (StateActionPtr const& col : columns) {
      // s must be either internal or a fringe node and:
      // (i) fringe implies a_fringe_or_deadend_ is defined
      assert(!isArtificialGoal(col.state)
              || occupancy_measure_.find({col.state, a_fringe_or_deadend_}) != occupancy_measure_.end());
      // (ii)  gpt::use_instability_sink implies a_fringe_or_deadend_ (for all states)
      assert(!gpt::use_deadend_sink
              || occupancy_measure_.find({col.state, a_fringe_or_deadend_}) != occupancy_measure_.end());
      assert(col.action_ptr != nullptr);

      if (isArtificialGoal(col.state)) {
        partiallyExpandArtificialGoal(col);
      }
      else {
        addInternalColumn(col);
      }

    }  // for all col (s,a,r) in columns

    // Done changing the model. Thus we can release the constrs (unique_ptr takes care of it) and
    // update the model_.
    model_->update();
  }

  double optDualVarForSinkConstr() const { return optDualVarFor("sink"); }
  double optDualVarForSourceConstr() const { return optDualVarFor("source"); }
  double optDualVarForCostConstr(size_t i) const { return optDualVarFor(costConstrName(i)); }
  double optDualVarForFlowConstr(state_t const& s) const {return optDualVarFor(flowConstrName(s));}

  double dualRayForSinkConstr() const { return dualRayFor("sink"); }
  double dualRayForSourceConstr() const { return dualRayFor("source"); }
  double dualRayForCostConstr(size_t i) const { return dualRayFor(costConstrName(i)); }
  double dualRayForFlowConstr(state_t const& s) const { return dualRayFor(flowConstrName(s)); }

  /*
   * Methods for Statistics
   */
  size_t numInternalStates() const { return internal_states_.size(); }
  size_t numArtificialGoals() const { return artificial_goals_.size(); }
  size_t numColsInRMP() const { return occupancy_measure_.size(); }
  size_t numColsAvailable() const { return columns_available_.size(); }
  size_t numGRBVars() const { return model_->get(GRB_IntAttr_NumVars); }
  size_t numGRBConstrs() const { return model_->get(GRB_IntAttr_NumConstrs); }

  // Return the number internal states in each of these partitions:
  //  - Trivial: states that are either goal, explicit dead ends, or have only 1 applicable action
  //  - Fully Expanded: states with more than 1 applicable action that have all actions expanded
  //  - Partially Expanded: all others, i.e., states with more than 1 applicable action that have
  //                        at least one non-expanded action
  std::tuple<size_t,size_t,size_t> numInternalStatesPerType() const;


  void saveLP(size_t ite) const;

 private:
  void buildBaseCase(bool use_instability_sink);

  SetStates newReachableStatesFrom(SetOfColumns const& columns);

  void addEmptyFlowConstrFor(state_t const& s) {
    assert(!hasFlowConstrFor(s));
    model_->addConstr(0.0, GRB_EQUAL, 0.0, flowConstrName(s));
  }

  void insertStateToSearchSpace(state_t const& s);
  void partiallyExpandArtificialGoal(StateActionPtr const& column);
  void addInternalColumn(StateActionPtr const& column);

  /*
   * Constraint Name Generators
   */
  std::string costConstrName(size_t cost_func_idx) const {
    assert(cost_func_idx < cssp_.constraints().size());
    return "cost_" + std::to_string(cost_func_idx);
  }
  std::string flowConstrName(state_t const& s) {
    auto it = flow_constr_name_.find(s);
    if (it != flow_constr_name_.end()) {
      return it->second;
    }
    std::string name("flow_s" + std::to_string(flow_constr_name_.size()));
    flow_constr_name_.insert(it, {s,name});
    return name;
  }
  std::string flowConstrName(state_t const& s) const {
    auto it = flow_constr_name_.find(s);
    assert(it != flow_constr_name_.end());
    return it->second;
  }


  /*
   * Constr GETTERs
   */
  GRBConstr getCostConstr(size_t cost_func_idx) {
    assert(cost_func_idx < cssp_.constraints().size());
    return model_->getConstrByName(costConstrName(cost_func_idx));
  }
  GRBConstr getFlowConstrFor(state_t const& s) {
    assert(hasFlowConstrFor(s));
    return model_->getConstrByName(flowConstrName(s));
  }
  GRBConstr getSinkConstr() { return model_->getConstrByName("sink"); }

  /*
   * Gurobi value getters
   */
  double optDualVarFor(std::string const& constr_name) const {
    assert(model_status_ == ModelStatus::OPTIMAL);
    return model_->getConstrByName(constr_name).get(GRB_DoubleAttr_Pi);
  }
  double dualRayFor(std::string const& constr_name) const {
    assert(model_status_ == ModelStatus::INFEASIBLE);
    return model_->getConstrByName(constr_name).get(GRB_DoubleAttr_FarkasDual);
  }

  /*
   * Private Member Variables
   */
  ConstrSSPIface const& cssp_;
  CachedVectorHeuristicWrapper& cached_h_vector_;
  action_t const* a_fringe_or_deadend_;

  enum class ModelStatus : int { NOT_OPTIMIZED, OPTIMAL, INFEASIBLE };
  ModelStatus model_status_;
  GRBModelSPtr model_;

  // Helper variable to make sink constraint add up to 1 when there is numerical instability. It
  // will be penalized more than the dead-end to make sure that gurobi won't take advantage of it
  // when not needed. If null, then the instability_sink is not being used.
  std::unique_ptr<GRBVar> instability_sink_;

  SetStates internal_states_;   // S' \setminus {G' \setminus G}, i.e., all non-artificial goals
  SetStates artificial_goals_;  // G' \setminus G
  SetOfColumns columns_available_;

  std::unordered_map<StateActionPtr, GRBVar> occupancy_measure_;
  // Stores the name of the flow constraint for each state
  std::unordered_map<state_t, std::string> flow_constr_name_;
};

#endif  // USE_GUROBI
#endif  // PLANNER_CG_DUAL_SEARCH_SPACE_H
