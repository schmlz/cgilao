#ifndef POLICY_EVALUATOR_H
#define POLICY_EVALUATOR_H

#include <fstream>
#include <iostream>

#include "../utils/lp_solver_wrapper.h"
#include "planner_iface.h"
using namespace LPWrappers;

// Helper function for checking if element x is in container c. (I find it
// very easy to make a mistake with the negations involved in the default find
// method.)
// * Note: currently this is only intended to work for unordered_map and
//   unordered_set
template <typename Container, typename ContainerT>
inline bool contains(Container const& c, ContainerT x)
{
  return c.find(x) != c.end();
}

/*
 * Struct containing the information of interest when evaluation a policy
 */
struct PolicyEval {
  double p_goal;
  double p_open;
  double p_max_actions;
  double p_other;
  double v_s0;
  void dump(std::string const& tag)
  {
    std::cout << "[" << tag << "] p_goal is " << p_goal << std::endl
              << "[" << tag << "] p_open is " << p_open << std::endl
              << "[" << tag << "] p_other is " << p_other << std::endl
              << "[" << tag << "] p_max_actions is " << p_max_actions << std::endl
              << "[" << tag << "] v_s0 is " << v_s0 << std::endl;
  }
  void dump_csv_entry(std::string const& filepath, const uint64_t timestamp,
                      std::string const& info) const
  {
    if (filepath == "") { return; }

    std::ofstream file;
    file.open(filepath, std::ios::app);
    file << timestamp << "," << p_goal << "," << p_open << "," << p_other << "," << v_s0 << ","
         << info << "\n";
    file.close();
  }
};

class PolicyEvaluator
{
 public:
  PolicyEvaluator(SSPIface const& ssp, const uint64_t planner_start_time,
                  const uint64_t evaluation_interval)
      : ssp_(ssp),
        planner_start_time_(planner_start_time),
        evaluation_interval_(evaluation_interval),
        total_evaluation_time_(0),
        last_evaluation_deadline_(planner_start_time_),
        next_evaluation_deadline_(last_evaluation_deadline_ + evaluation_interval_)
  {
    // Print header of csv (if filepath exists)
    //
    // NOTE: overwrites previous CSV!
    if (gpt::online_test_csv_filepath != "") {
      std::ofstream file;
      file.open(gpt::online_test_csv_filepath);
      file << "time_usecs,p_goal,p_open,p_other,v_s0,info\n";
      file.close();
    }
  }

  ~PolicyEvaluator() {}

  /******************
   * Helper methods
   ******************/

  enum class StateType { GOAL, OPEN, OTHER, DEFINED_ACTION };

  std::tuple<StateType, action_t const*> actionAt(Planner const& planner, state_t const& s) const
  {
    if (ssp_.isGoal(s)) { return {StateType::GOAL, nullptr}; }

    try {
      action_t const* const a_ptr = planner.decideAction(s);
      if (a_ptr == nullptr) {
        return {StateType::OTHER, a_ptr};
      } else {
        return {StateType::DEFINED_ACTION, a_ptr};
      }
    } catch (const PlannerGaveUpException& e) {
      return {StateType::OPEN, nullptr};
    } catch (...) {
      std::cerr << "SOMETHING WENT WRONG IN < planner.decideAction(s) >\n";
      EXIT("BAD POLICY");
      return {};
    }
  }

  /******************
   * Evaluation code
   ******************/

  std::unordered_set<state_t> policyEnvelope(Planner const& planner) const
  {
    if (gpt::verbosity > 0) {
      std::cout << "[policy-eval-lp]: Computing policy envelope..." << std::endl;
    }

    // Compute the states in the policy envelope by traversing with DFS
    std::unordered_set<state_t> envelope = {ssp_.s0()};
    std::vector<state_t> frontier        = {ssp_.s0()};
    while (not frontier.empty()) {
      const state_t current_state = frontier.back();
      frontier.pop_back();

      const auto [state_type, a_ptr] = actionAt(planner, current_state);

      if (state_type != StateType::DEFINED_ACTION) { continue; }
      assert(a_ptr != nullptr);

      static ProbDistStateHash pr_s_a;
      ssp_.expand(*a_ptr, current_state, pr_s_a);
      for (auto const& ip : pr_s_a) {
        if (not contains(envelope, ip.event())) {
          frontier.push_back(ip.event());
          envelope.emplace(ip.event());
        }
      }
    }

    if (gpt::verbosity > 0) {
      std::cout << "[policy-eval-lp]: ...done computing the policy envelope" << std::endl;
    }

    return envelope;
  }

  void setupMaxProbLP(Planner const& planner, std::unordered_set<state_t> const& pi_envelope)
  {
    // Set problem to MAXIMISATION -- we want to maximise the prob. of reaching goal
    {
      auto row = solver_->row();
      solver_->set_maximisation_objective(row);
    }

    // Generate empty conservation flow constraint for all s in the envelope
    for (state_t const& s : pi_envelope) {
      // add empty constraint
      auto row                   = solver_->row();
      const double source_inflow = (ssp_.s0() == s) ? 1.0 : 0.0;
      flow_conservation_[s]      = solver_->add_constraint(row <= source_inflow);
    }

    // Define give-up variables
    for (state_t const& s : pi_envelope) {
      // get the type of this state
      const auto state_type = std::get<StateType>(actionAt(planner, s));

      // any give-up contributes to the out-flow of a state
      Column new_column = solver_->column();
      solver_->add_to_column(new_column, 1.0, flow_conservation_.at(s));

      if (state_type == StateType::GOAL) {
        expected_usage_.giveup_goal[s] = solver_->add_variable(0,            // lower bound
                                                               infinity,     // upper bound
                                                               1.0,          // sign in obj func
                                                               new_column);  // column
      } else if (state_type == StateType::OPEN) {
        expected_usage_.giveup_open[s] = solver_->add_variable(0,            // lower bound
                                                               infinity,     // upper bound
                                                               1.0,          // sign in obj func
                                                               new_column);  // column
      }
    }

    // Set up flow variables and their constraints
    //
    // NOTE: pi may be stochastic, so pi(s) refers to to a distribution of actions, and so we need
    // to think about P(eff | s, pi(s)) as the probability of reaching eff over all actions we may
    // apply in s
    for (state_t const& s : pi_envelope) {
      const auto [state_type, a_ptr] = actionAt(planner, s);
      if (state_type != StateType::DEFINED_ACTION) { continue; }
      assert(a_ptr != nullptr);

      // count up the probability of reaching each possible outcome of pi(s)
      //
      // FIXME(jsch): this is a remnant from stochastic policies, really overkill for det. pi
      std::unordered_map<state_t, double> prob;
      static ProbDistStateHash pr_s_a;
      ssp_.expand(*a_ptr, s, pr_s_a);
      for (auto const& ip : pr_s_a) { prob[ip.event()] += ip.prob(); }

      Column new_column = solver_->column();

      // contribution to s
      //
      // NOTE: if there is a self-loop then we need to deal with the in-flow from the action here!
      solver_->add_to_column(new_column, 1.0 - prob[s], flow_conservation_.at(s));

      // contribution to effects
      for (auto const& [eff, total_prob_eff] : prob) {
        if (eff == s) { continue; }
        // x_{s, pi(s)} contributes -P(eff | s, pi(s)) to in(eff)
        solver_->add_to_column(new_column, -total_prob_eff, flow_conservation_.at(eff));
      }

      // add the variable
      expected_usage_.pi_s[s] = solver_->add_variable(0,            // lower bound
                                                      infinity,     // upper bound
                                                      0.0,          // sign in obj func
                                                      new_column);  // column
    }
  }

  void turnMaxProbIntoMCMP(Planner const& planner, const double p_max)
  {
    // Add constraint in(G) == p_max
    auto goal_inflow = solver_->row();
    for (auto const& [s, var_s_gu_g] : expected_usage_.giveup_goal) {
      solver_->add_to_row(goal_inflow, var_s_gu_g);
    }
    solver_->add_constraint(goal_inflow == p_max);

    // Change objective to cost of actions
    auto action_obj_contribution = solver_->row();
    for (auto const& [s, var_s_a] : expected_usage_.pi_s) {
      const auto [s_type, a_ptr] = actionAt(planner, s);
      assert(s_type == StateType::DEFINED_ACTION);
      assert(a_ptr != nullptr);
      solver_->add_to_row(action_obj_contribution, var_s_a * ssp_.cost(s, *a_ptr).double_value());
    }
    solver_->set_minimisation_objective(action_obj_contribution);
  }

  PolicyEval evaluatePolicyWithLP(Planner const& planner)
  {
    // Set up and solve the LP
    //
    // TODO(jsch): I am constructing a new LPWrap object here, which means a new gurobi/cplex
    // environment will be set up, which is inefficient!
    solver_.reset(new LPWrap());
    expected_usage_      = ExpectedUsage();
    flow_conservation_   = FlowConstrMap();
    auto policy_envelope = policyEnvelope(planner);

    PolicyEval eval = {};

    // Compute probabilities with MAXPROB LP
    setupMaxProbLP(planner, policy_envelope);
    solver_->solve();
    assert(solver_->solved_optimally());
    double p_goal = 0.0;
    double p_open = 0.0;
    double p_other = 0.0;
    for (auto const& [s, var_s_gu_g] : expected_usage_.giveup_goal) {
      assert(ssp_.isGoal(s));
      p_goal += solver_->get_variable_assignment(var_s_gu_g);
    }
    for (auto const& [s, var_s_gu_o] : expected_usage_.giveup_open) {
      p_open += solver_->get_variable_assignment(var_s_gu_o);
    }
    // Giving up at a non-open state is encoded by the slack variables of our conservation-of-flow
    // inequalities
    for (auto const& [s, flow_constr_s] : flow_conservation_) {
      p_other += solver_->get_constr_slack(flow_constr_s);
    }
    eval.p_goal = p_goal;
    eval.p_open = p_open;
    eval.p_other = p_other;
    assert(abs(1.0 - eval.p_goal - eval.p_open - eval.p_other) < gpt::epsilon);

    // Compute costs with MCMP LP using probability from MAXPROB LP
    turnMaxProbIntoMCMP(planner, eval.p_goal);
    solver_->solve();
    assert(solver_->solved_optimally());
    double v_s0 = 0.0;
    for (auto const& [s, var_s_a] : expected_usage_.pi_s) {
      const auto [s_type, a_ptr] = actionAt(planner, s);
      assert(s_type == StateType::DEFINED_ACTION);
      assert(a_ptr != nullptr);
      v_s0 += ssp_.cost(s, *a_ptr).double_value() * solver_->get_variable_assignment(var_s_a);
    }
    eval.v_s0 = v_s0;

    eval.dump("LP");
    return eval;
  }

  struct SamplingData {
    size_t n_reached_goal;
    size_t n_open;
    size_t n_others;
    size_t n_max_action;
    std::vector<double> sample_cost;
  };

  PolicyEval evaluatePolicyWithSampling(Planner const& planner, size_t n_samples,
                                        size_t max_actions = 1000)
  {
    // Run simulation and collect data
    SamplingData data = {};
    for (size_t i = 0; i < n_samples; ++i) {
      state_t s        = ssp_.s0();
      size_t n_actions = 0;
      double acc_cost  = 0;

      while (true) {
        const auto [state_type, a_ptr] = actionAt(planner, s);

        if (state_type == StateType::GOAL) {
          assert(ssp_.isGoal(s));
          data.n_reached_goal++;
          break;
        } else if (state_type == StateType::DEFINED_ACTION) {
          a_ptr->affect(s);
          acc_cost += ssp_.cost(s, *a_ptr).double_value();
          n_actions++;
        } else if (state_type == StateType::OPEN) {
          data.n_open++;
          break;
        } else if (state_type == StateType::OTHER) {
          data.n_others++;
          break;
        } else {
          NOT_IMPLEMENTED;
        }

        if (n_actions >= max_actions) {
          data.n_max_action++;
          break;
        }
      }
      data.sample_cost.push_back(acc_cost);
    }

    // Process data into info about pi
    PolicyEval results;
    results.p_goal        = static_cast<double>(data.n_reached_goal) / n_samples;
    results.p_open        = static_cast<double>(data.n_open) / n_samples;
    results.p_other       = static_cast<double>(data.n_others) / n_samples;
    results.p_max_actions = static_cast<double>(data.n_max_action) / n_samples;

    double avg_cost = 0;
    for (double c : data.sample_cost) { avg_cost += c; }
    avg_cost /= n_samples;
    results.v_s0 = avg_cost;

    results.dump("sampling");
    return results;
  }

  void evaluatePolicyOnTimer(Planner const& planner, std::string const& info,
                             const bool force_evaluation = false)
  {
    const uint64_t eval_start = get_cputime_usec();

    // If evaluation is not forced, check if we have reached the evaluation deadline yet
    if (eval_start < next_evaluation_deadline_ and not force_evaluation) {
      total_evaluation_time_ += get_cputime_usec() - eval_start;
      return;
    }

    // Get the planner's runtime i.e. how long the planner has been running without any of the
    // evaluation code --- this is the timestamp associated with this evaluation in the CSV
    const uint64_t planner_time = eval_start - planner_start_time_ - total_evaluation_time_;

    // Do evaluation
    const auto results_lp = evaluatePolicyWithLP(planner);
    results_lp.dump_csv_entry(gpt::online_test_csv_filepath, planner_time, info);

    const auto results_sampling = evaluatePolicyWithSampling(planner, 10000, 200);
    results_sampling.dump_csv_entry(gpt::online_test_csv_filepath, planner_time,
                                    info + " (SAMPLING)");

    // Wrap up timer after evaluation
    last_evaluation_deadline_ = next_evaluation_deadline_;
    const uint64_t eval_stop  = get_cputime_usec();
    total_evaluation_time_ += eval_stop - eval_start;

    // Relative timer
    next_evaluation_deadline_ = eval_stop + evaluation_interval_;

    // // Absolute timer
    // //
    // // Set next deadline to n * evaluation_interval_ + planner_start_time_ where n is the
    // smallest
    // // nat. so that this timestamp is in the future
    // while (eval_stop >= next_evaluation_deadline_) {
    //   next_evaluation_deadline_ += evaluation_interval_;
    // }
  }

  uint64_t getEvaluationTime() const { return total_evaluation_time_; }

 private:
  SSPIface const& ssp_;

  // Timer vars
  const uint64_t planner_start_time_;
  const uint64_t evaluation_interval_;
  uint64_t total_evaluation_time_;
  uint64_t last_evaluation_deadline_;
  uint64_t next_evaluation_deadline_;

  // Setting up which solver we're using
  #if defined USE_GUROBI
  using LPWrap = LPWrapper<LPSolver::GUROBI>;
  #elif defined USE_CPLEX
  using LPWrap    = LPWrapper<LPSolver::CPLEX>;
  #endif
  using Var     = LPWrap::Var;
  using LinExpr = LPWrap::Row;
  using Column  = LPWrap::Column;
  using Constr  = LPWrap::Constr;

  // Helper defn.s
  using FlowConstrMap = std::unordered_map<state_t, Constr>;
  struct ExpectedUsage {
    std::unordered_map<state_t, Var> pi_s;
    std::unordered_map<state_t, Var> giveup_open;
    std::unordered_map<state_t, Var> giveup_goal;
  };

  // Main variables
  std::unique_ptr<LPWrap> solver_;
  FlowConstrMap flow_conservation_;
  ExpectedUsage expected_usage_;

  // Some shorthand variables
  const double deadend_value = gpt::dead_end_value.double_value();
  const double infinity      = std::numeric_limits<double>::infinity();
};

#endif  // POLICY_EVALUATOR_H
