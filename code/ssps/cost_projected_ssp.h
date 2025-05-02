#ifndef CSSP_IFACE_H
#define CSSP_IFACE_H

#include "ssp_iface.h"

class CostProjectedSSP : public SSPIface {
 public:
  CostProjectedSSP(ConstrSSPIface const& cssp, size_t cost_idx)
    : cssp_(cssp), cost_idx_(cost_idx), name_("Test")
  { }

  // It was already a method from problem_t
  std::string const& name() const override { return name_; }

  state_t const& s0() const override { return cssp_.s0(); }

  bool isGoal(state_t const& s) const override { return cssp_.isGoal(s); }

  bool hasApplicableActions(state_t const& s) const override {
    auto range = cssp_.applicableActions(s);
    return range.begin() != range.end();
  }

  bool isApplicable(state_t const& s, action_t const& a) const override {
    return !isGoal(s) && a.enabled(s);
  }

  ActionConstRange applicableActions(state_t const& s) const override {
    return cssp_.applicableActions(s);
  }

  void expand(action_t const& a, state_t const& s, ProbDistStateIface& pr)
    const override
  {
    cssp_.expand(a, s, pr);
  }

  Rational cost(state_t const& s, action_t const& a) const override {
    return a.cost(s, cost_idx_);
  }

  Rational terminalCost(state_t const& s) const override {
    return cssp_.terminalCost(s);
  }

  StateConstRange reachableStates() const override {
    return wrappedReachableStates(*this, s0());
  }

 private:
  ConstrSSPIface const& cssp_;
  size_t cost_idx_;
  std::string name_;
};

#endif  // CSSP_IFACE_H
