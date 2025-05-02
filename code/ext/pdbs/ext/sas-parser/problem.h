#ifndef EXT_SAS_PARSER_PROBLEM_PDB
#define EXT_SAS_PARSER_PROBLEM_PDB

#include <iostream>
#include <vector>

#include "state.h"
#include "mutex_group.h"
#include "operator.h"
#include "axiom.h"
#include "variable.h"
#include "helper_functions.h"

// helper class to wrap everything together

namespace FastDownwardParser_PDB {

struct SasProblem {
  SasProblem() { }
  SasProblem(std::istream& input_s) {
    read_preprocessed_problem_description(input_s, metric, internal_variables,
                   variables, mutexes, initial_state, goals, operators, axioms);
  }

  ~SasProblem() { }

  void dump() const {
    dump_preprocessed_problem_description(variables, initial_state, goals,
                                                             operators, axioms);
  }

  bool metric;
  std::vector<Variable*> variables;
  std::vector<Variable> internal_variables;
  State initial_state;
  std::vector<pair<Variable*, int>> goals;
  std::vector<MutexGroup> mutexes;
  std::vector<Operator> operators;
  std::vector<Axiom> axioms;
};

}

#endif  // EXT_SAS_PARSER_PROBLEM
