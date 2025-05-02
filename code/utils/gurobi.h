#ifndef UTILS_GUROBI_H
#define UTILS_GUROBI_H

#include <iostream>
#include <memory>

#if defined USE_GUROBI

#include "gurobi_c++.h"

using GRBEnvSPtr   = std::shared_ptr<GRBEnv>;
using GRBModelSPtr = std::shared_ptr<GRBModel>;

namespace Gurobi {

extern GRBEnvSPtr env_;


void initEnvironment();

inline GRBEnvSPtr sharedEnv() { return env_; }

GRBModelSPtr newModel();

struct SelectAllValues {
  bool operator()(double) const { return true; }
};

struct SelectGreaterThanZero {
  bool operator()(double x) const { return x > 0; }
};

template<typename F = SelectAllValues>
void printSolution(GRBModel const& model) {
  F select_func;
  GRBVar* vars = model.getVars();
  for (int i = 0; i < model.get(GRB_IntAttr_NumVars); ++i) {
    double value = vars[i].get(GRB_DoubleAttr_X);
    if (select_func(value)) {
      std::cout << vars[i].get(GRB_StringAttr_VarName) << " = " << value << std::endl;
    }
  }
  delete[] vars;
}

inline void printVarsGreaterThanZero(GRBModel const& model) {
  printSolution<SelectGreaterThanZero>(model);
}

// Simple function to return gurobi's error code explanation. The messages were
// obtained from:
// http://www.gurobi.com/documentation/6.5/refman/optimization_status_codes.html
std::string errorCodeTranslation(int error_code);


};

#endif  // USE_GUROBI
#endif  // UTILS_GUROBI_H
