#ifndef HEURISTICS_DEAD_END_H
#define HEURISTICS_DEAD_END_H

#include <iostream>

#include "heuristic_iface.h"
#include "heuristic_cssp_iface.h"
#include "h_max.h"

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/states.h"


class DeadendHeuristic : public heuristic_t {
 public:
  DeadendHeuristic(problem_t const& problem)
    : heuristic_t("dead-end-from-h-max"), h_max_(problem, ACTION_COST),
      non_dead_end_h_(nullptr)
  { }

  DeadendHeuristic(problem_t const& problem, heuristic_t* non_dead_end_h)
    : heuristic_t("dead-end-from-h-max:" + non_dead_end_h->name()),
      h_max_(problem, ACTION_COST), non_dead_end_h_(non_dead_end_h)
  { }

  virtual ~DeadendHeuristic() {
    delete non_dead_end_h_;
  }

 protected:
  double computeValue(state_t const& s) {
    if (h_max_.value(s) >= gpt::dead_end_value.double_value())
      return gpt::dead_end_value.double_value();
    else if (non_dead_end_h_)
      return non_dead_end_h_->value(s);
    else
      return 0.0;
  }

 private:
  HMaxAllOutcomesDet h_max_;
  heuristic_t* non_dead_end_h_;
};



class VecDeadendHeuristic : public TimedHeuristicCSSPIface {
 public:
  VecDeadendHeuristic(problem_t const& problem)
    : TimedHeuristicCSSPIface(), h_max_(problem, ACTION_COST),
      dead_end_vector_(problem.numCostFunctions(), -1.0)
  { }

  VecDeadendHeuristic(problem_t const& problem, HeuristicCSSPUniqPtr non_dead_end_h)
    : TimedHeuristicCSSPIface(), h_max_(problem, ACTION_COST),
      dead_end_vector_(problem.numCostFunctions(), -1.0),
      non_dead_end_h_(std::move(non_dead_end_h))
  {
    initDeadendVec(problem);
  }

  virtual ~VecDeadendHeuristic() { statistics(); }

  std::string name() const override {
    if (non_dead_end_h_)
      return "vec-dead-end:" + non_dead_end_h_->name();
    else
      return "vec-dead-end";
  }


 protected:
  void computeValueVec(state_t const& s, std::vector<double>& h_vec) override {
    if (h_max_.value(s) >= gpt::dead_end_value.double_value())
      h_vec = dead_end_vector_;
    else if (non_dead_end_h_)
      non_dead_end_h_->valueVec(s, h_vec);
    else {
      for (double& v : h_vec)
        v = 0.0;
    }
  }


 private:
  void initDeadendVec(problem_t const& problem) {
    dead_end_vector_[ACTION_COST] = gpt::dead_end_value.double_value();
    ConstraintMap const& cost_constraints = problem.constraints();
    for (size_t i = 0; i < cost_constraints.size(); ++i) {
      dead_end_vector_[cost_constraints.costIdx(i)] = cost_constraints.deadendPenalty(i);
    }
  }


  HMaxAllOutcomesDet h_max_;
  std::vector<double> dead_end_vector_;
  HeuristicCSSPUniqPtr non_dead_end_h_;
};

#endif  // HEURISTICS_DEAD_END_H
