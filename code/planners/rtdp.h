#ifndef PLANNER_RTDP_H
#define PLANNER_RTDP_H

#include <iostream>

#include "planner_iface.h"

#include "../ext/mgpt/actions.h"
#include "../ssps/bellman.h"
#include "../ext/mgpt/hash.h"
#include "../ssps/policy.h"
#include "../utils/utils.h"

class SSPIface;
class heuristic_t;


/*******************************************************************************
 *
 * planner RTDP: Real Time Dynamic Programming
 *
 ******************************************************************************/
class PlannerRTDP : public HeuristicPlanner {
 public:
  PlannerRTDP(SSPIface const& ssp, heuristic_t& heur, double epsilon);
  virtual ~PlannerRTDP() { }

  /*
   * Planner Interface
   */
  action_t const* decideAction(state_t const& s) override {
    trial();
    double min_q_value = 0;
    action_t const* a_greedy = nullptr;
    std::tie(a_greedy, min_q_value) =
                                Bellman::greedyActionAndMinQValue(s, v_, ssp_);
    v_.update(s, min_q_value);
    return a_greedy;
  }

  action_t const* decideAction(state_t const& s) const override {
    return Bellman::constGreedyAction(s, v_, ssp_);
  }

  void trainForUsecs(uint64_t max_time_usec) override {
    // The return value of runForUsec is ignored because we will run as many
    // trials as possible.
    //
    runForUsec(max_time_usec,
               // lambda function that captures this and runs trial()
               [this]() { while(true) trial(); });
  }

  void initRound() override { }
  void endRound() override { }
  void resetRoundStatistics() override { };
  void statistics(std::ostream& os, int level) const override;

  /*
   * Heuristic Planner Interface
   */
  double value(state_t const& s) const override { return v_.value(s); }

 private:
  // RTDP trial method from the initial state s0.
  void trial();

  /*
   * Member variables
   */
  SSPIface const& ssp_;
  hash_t v_;
  double epsilon_;
};

#endif  // PLANNER_RTDP_H
