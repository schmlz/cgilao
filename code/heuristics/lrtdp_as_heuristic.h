#ifndef HEURISTICS_LRTDP_H
#define HEURISTICS_LRTDP_H

#include <iostream>
#include <random>

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/rational.h"
#include "../ext/mgpt/states.h"
#include "../planners/lrtdp.h"
#include "../ssps/ssp_iface.h"
#include "../utils/die.h"
#include "heuristic_iface.h"

/*******************************************************************************
 *
 * LRTDP heuristic
 *
 ******************************************************************************/
class LRTDPHeuristic : public heuristic_t
{
 public:
  LRTDPHeuristic(SSPIface const& ssp, heuristic_t& heur, const double perturbation_lb,
                 double epsilon)
      : heuristic_t("LRTDP_as_heuristic"),
        lrtdp_(ssp, heur, epsilon),
        perturbation_lb_(perturbation_lb)
  {
    // do not print CSVHACK for each call!
    lrtdp_.setPrintFinishedInfo(false);
  }
  ~LRTDPHeuristic() {}

  /*
   * heuristic_t interface
   */
  double computeValue(state_t const& s)
  {
    // std::cout << "[lrtdp-as-heuristic] solving for " << s.toString() << "..." << std::endl;

    // HACK(jsch): used later
    const size_t n_qvalues_before = gpt::total_computed_qvalues;

    // decideAction makes sure that the problem is solved from s
    lrtdp_.decideAction(s);

    // std::cout << "[lrtdp-as-heuristic] ...done." << std::endl;

    // HACK(jsch): ignore the q-values accumulated by LRTDP as heuristic
    gpt::total_computed_qvalues = n_qvalues_before;

    return lrtdp_.value(s) * getPerturbationWeight(s);
  }

  /**
   * @brief Returns a pseudo-random weight uniformly distributed across (perturbation_lb_, 1.0]
   *        using gpt::seed XOR s.code as the seed
   *
   * @param s
   * @return double in (perturbation_lb_, 1.0]
   */
  double getPerturbationWeight(state_t const& s) const
  {
    // Set the seed of this particular state
    auto const seed = s.code() ^ gpt::seed;

    // Set up new generator and distribution
    //
    // NOTE: both may be stateful
    // (https://quuxplusone.github.io/blog/2019/10/22/psa-stateful-distributions/)
    //
    // NOTE: we can probably do generator.seed(seed) and distribution.reset() to reset their
    // states and not reconstruct
    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> distribution(0.0, 1.0 - perturbation_lb_);

    // Generate 1-w in [0, 1-perturbation_lb_)
    //
    // NOTE: if we get distribution [perturbation_lb_, 1.0) then it is undefined for
    // perturbation_lb_ = 1.0, that's why we do 1-w
    const double one_minus_w = distribution(generator);

    return 1.0 - one_minus_w;
  }

 private:
  PlannerLRTDP lrtdp_;
  const double perturbation_lb_;
};

#endif  // HEURISTICS_LRTDP_H
