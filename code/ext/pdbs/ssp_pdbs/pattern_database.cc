#include "pattern_database.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "match_tree.h"
#include "../algs/ssp_solvers.h"

using namespace std;

namespace ssp_pdbs {

bool is_product_within_limit(int factor1, int factor2, int limit) {
  assert(factor1 >= 0);
  assert(factor2 >= 0);
  assert(limit >= 0);
  return factor2 == 0 || factor1 <= limit / factor2;
}

PatternDatabase::PatternDatabase(const Problem& problem, const Pattern& pattern, const vector<Cost>& operator_costs)
    : pattern(pattern) {
  assert(operator_costs.empty() ||
         operator_costs.size() == problem.allActions().size());
  assert(std::is_sorted(pattern.begin(), pattern.end()));

  vector<Variable> const& variables = problem.variables();
  variable_to_index = vector<int>(variables.size(), -1);
  for (size_t i = 0; i < pattern.size(); ++i) {
    variable_to_index[pattern[i]] = i;
  }

  num_objectives = problem.numCostFunctions();

  hash_multipliers.reserve(pattern.size());
  num_states = 1;
  for (int pattern_var_id : pattern) {
    hash_multipliers.push_back(num_states);
    Variable var = problem.variable(pattern_var_id);
    if (is_product_within_limit(num_states, var.get_domain_size(),
                                numeric_limits<int>::max())) {
      num_states *= var.get_domain_size();
    } else {
      cerr << "Given pattern is too large! (Overflow occured)" << endl;
      std::exit(1);
    }
  }

  distances = vector<double>(num_states);

  std::cout << "Computing PDB for pattern ";
  for (auto const x : pattern) { std::cout << x << "  "; }
  std::cout << std::endl;


  create_pdb(problem, operator_costs);


  std::cout << "Finished computing PDB for pattern ";
  for (auto const x : pattern) { std::cout << x << "  "; }
  std::cout << std::endl;
}



void PatternDatabase::create_pdb(const Problem& problem, const vector<Cost>& operator_costs) {

  /* Step 1: create abstract problem */

  // Abstract variables
  vector<SasPlus::SasPlusVariable> abs_variables;
  for (size_t i = 0; i < pattern.size(); ++i) {
    abs_variables.push_back(problem.variables()[pattern[i]]);
  }

  // Abstract initial state
  SasPlus::SasPlusState p_s0 = project(problem.initialState());

  // Abstract goal
  SasPlus::SasPlusPartialState p_goal = project(problem.goal());

  // Abstract operators
  std::set<Action> p_actions_set;
  std::vector<Action> p_actions;
  for (auto const &action: problem.allActions()) {
    std::string a_name = action.name()+"-abstract";
    Cost cost = action.cost();
    SasPlus::SasPlusPartialState prec = project(action.precondition());

    std::map<SasPlus::SasPlusPartialState, double> pr_effect_dist;
    for (size_t i = 0; i < action.pr_effects().size(); i++) {
      SasPlus::SasPlusPartialState p_effect = project(action.pr_effects()[i]);
      if (!pr_effect_dist.contains(p_effect)) {
        pr_effect_dist[p_effect] = 0;
      }
      pr_effect_dist[p_effect] += action.pr_dist()[i];
    }
    SasPlus::SasPlusAction::PrEffect pr_effects;
    SasPlus::SasPlusAction::PrDist pr_dist;
    bool non_empty_effect = false;
    for (auto const &kv: pr_effect_dist) {
      if (kv.first.size() > 0) {
        non_empty_effect = true;
      }
      pr_effects.push_back(kv.first);
      pr_dist.push_back(kv.second);
    }

    if (non_empty_effect) {
      p_actions_set.insert({a_name, cost, prec, pr_effects, pr_dist});
    }
  }
  for (auto const &a: p_actions_set) {
    p_actions.push_back(a);
  }
  if (p_actions.empty()) {
    return;  // everything has 0 cost
  }

  SasPlus::SasPlusMOSSP abs_problem = {
          problem.name()+"-abstract",
          abs_variables,
          p_s0,
          p_goal,
          p_actions
  };



  /* Step 2: solve abstract problem with VI/TVI */
  auto v = Ssp_Solvers_For_Mossp::tvi(abs_problem, gpt::epsilon);


  /* Step 3: collect abstract heuristic values into a vector */
  for (auto const& [abs_state, h]: v) {
    int index = 0;
    assert(!pattern.empty());
    assert(abs_state.size() == pattern.size());
    for (size_t i = 0; i < pattern.size(); ++i) {
      index += hash_multipliers[i] * abs_state[i];
    }
    assert(0 <= index && index < num_states);
//    std::cout<<"index: "<<index<<std::endl;
//    problem.print(abs_state);
//    util::print(h);
    distances[index] = h;
  }
}

int PatternDatabase::hash_index(const State& state) const {
  int index = 0;
  assert(!pattern.empty());
  for (size_t i = 0; i < pattern.size(); ++i) {
    index += hash_multipliers[i] * state[pattern[i]];
  }
  return index;
}

double PatternDatabase::get_value(const State& state) const {
//  fmt::print("pattern: {}\n",
//             fmt::join(pattern, ","));
//  std::cout<<"index: "<<hash_index(state)<<std::endl;
//  for (size_t i = 0; i < state.size(); ++i) {
//    std::cout<<fmt::format("({},{}) ", i, state[i])<<std::endl;
//  }
//  util::print(distances[hash_index(state)]);
  return distances[hash_index(state)];
}

SasPlus::SasPlusState PatternDatabase::project(SasPlus::SasPlusState state) {
  SasPlus::SasPlusState p_state;
  for (size_t i = 0; i < pattern.size(); ++i) {
    p_state.push_back(state[pattern[i]]);
  }
  return p_state;
}

SasPlus::SasPlusPartialState PatternDatabase::project(SasPlus::SasPlusPartialState state) {
  SasPlus::SasPlusPartialState p_state;
  for (size_t i = 0; i < pattern.size(); ++i) {
    if (state.contains(pattern[i])) {
      p_state[i] = state[pattern[i]];
    }
  }
  return p_state;
}

} // namespace pdbs
