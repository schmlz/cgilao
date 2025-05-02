#ifndef SSP_SOLVERS_FOR_MOSSP_H
#define SSP_SOLVERS_FOR_MOSSP_H

#include <queue>
#include "../representations/sasplus.h"
#include "../../mgpt/global.h"

namespace Ssp_Solvers_For_Mossp {

using Problem = typename SasPlus::SasPlusMOSSP;
using State = typename Problem::State;
using Action = typename Problem::Action;

// Returns residual
double bellman_backup(State const& s, std::map<State, double>& v, Problem const& problem) {

  if (problem.isGoal(s)) {
    v[s] = 0.0;
    return 0.0;
  }

  double const v_s_before = v[s];
  double min_q = gpt::dead_end_value.double_value();
  for (auto& a : problem.applicableActions(s)) {
    // HACK: just using first cost of vector
    double q_s_a = a.cost()[0];
    for (auto& [eff, prob] : a.successors_prob(s)) {
      // IMPORTANT: for unseen states, v[eff] is initialised to 0.0
      q_s_a += v[eff] * prob;
    }
    min_q = std::min(min_q, q_s_a);
  }
  v[s] = min_q;
  return std::abs(v[s] - v_s_before);
}

std::map<State, double> vi(Problem const& problem, double const epsilon) {
  std::map<State, double> v;

  // Get reachable states
  State s_I = problem.initialState();
  std::set<State> vis;
  std::vector<State> ordered_vis;
  std::queue<State> queue;
  queue.push(s_I);
  vis.insert(s_I);
  ordered_vis.emplace_back(s_I);
  while (!queue.empty()) {
    State s = queue.front();
    queue.pop();
    if (problem.isGoal(s)) {
      continue;
    }
    for (const Action& a: problem.applicableActions(s)) {
      for (const State& s2: problem.successors(s, a)) {
        if (!vis.contains(s2)) {
          queue.push(s2);
          vis.insert(s2);
          ordered_vis.emplace_back(s2);
        }
      }
    }
  }

  // Apply Bellman backups
  std::reverse(ordered_vis.begin(), ordered_vis.end());
  double residual = 1.0 + epsilon;
  while (residual > epsilon) {
    residual = 0.0;
    for (State const& s : ordered_vis) {
      double const s_residual = bellman_backup(s, v, problem);
      residual = std::max(residual, s_residual);
    }
  }

  return v;
}


std::map<State, double> tvi(Problem const& problem, double const epsilon) {

  // Step 1: collect all reachable states

  std::map<State, std::set<State>> G_in;
  std::map<State, std::set<State>> G_out;

  State s_I = problem.initialState();
  std::vector<State> L;
  std::stack<State> dfs_stack;
  dfs_stack.push(s_I);
  G_in[s_I] = std::set<State>();
  while (!dfs_stack.empty()) {
    State state = dfs_stack.top();
    dfs_stack.pop();
    L.push_back(state);
    if (problem.isGoal(state)) {
      G_in[state] = std::set<State>();
      continue;
    }
    for (const Action& action: problem.applicableActions(state)) {
      for (const State& state2: problem.successors(state, action)) {
        G_in[state].insert(state2);
        if (!G_in.contains(state2)) {
          G_in[state2] = std::set<State>();
          dfs_stack.push(state2);
        }
      }
    }
  }

  // G_in: u -> v
  // G_out: v -> u
  for (const auto& kv: G_in) {
    State u = kv.first;
    for (const State& v: kv.second) {
      if (!G_out.contains(v)) {
        G_out[v] = std::set<State>();
      }
      G_out[v].insert(u);
    }
  }


  // Step 2: compute strongly connected components https://en.wikipedia.org/wiki/Kosaraju%27s_algorithm
  // linear in number of nodes
  std::vector<std::set<State>> sccs;  // scc containing s_I is at the front of the vector
  std::set<State> assigned;  // whether state has been assigned to an scc
  size_t size_largest_scc = 0;
  for (size_t i = 0; i < L.size(); i++) {
    State const s_top = L[i];
    if (assigned.contains(s_top)) {
      continue;
    }
    std::set<State> scc;
    std::stack<State> to_explore;
    to_explore.push(s_top);

    scc.insert(s_top);
    assigned.insert(s_top);
    while (!to_explore.empty()) {
      State const s = to_explore.top();
      to_explore.pop();
      for (const auto& s2: G_out[s]) {
        if (!assigned.contains(s2)) {
          to_explore.push(s2);
          scc.insert(s2);
          assigned.insert(s2);
        }
      }
    }

    size_largest_scc = std::max(size_largest_scc, scc.size());
    sccs.push_back(scc);
  }



  // Step 3: topological VI
  std::map<State, double> v;
  std::set<State>* scc;
  for (size_t i = 0; i < sccs.size(); i++) {
    scc = &sccs[sccs.size() - 1 - i];  // because scc[0] contains s_I

    double residual = 1 + epsilon;
    while (residual > epsilon) {

      residual = 0.0;
      for (const State& s: *scc) {
        double const s_residual = bellman_backup(s, v, problem);
        residual = std::max(residual, s_residual);
      }

    }
  }

  return v;
}

};

#endif // SSP_SOLVERS_FOR_MOSSP_H