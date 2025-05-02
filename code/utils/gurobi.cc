#include <iostream>
#include "../ext/mgpt/global.h"

# if defined USE_GUROBI

#include "gurobi_c++.h"
#include "gurobi.h"


namespace Gurobi {

GRBEnvSPtr env_;


void initEnvironment() {
  assert(env_ == nullptr);
  try {
    env_.reset(new GRBEnv);
    // Setting the defaults
    env_->set(GRB_IntParam_Seed, gpt::seed);
    std::cout << "[GUROBI]: using " << gpt::grb_nthreads
              << " threads. Use --grb_nthreads to change it.\n";
    env_->set(GRB_IntParam_Threads, gpt::grb_nthreads);
    env_->set(GRB_DoubleParam_FeasibilityTol, gpt::grb_feasibility_tol);
    env_->set(GRB_IntParam_Method, gpt::grb_method);
    env_->set(GRB_IntParam_NumericFocus, gpt::grb_numerical_focus);
    env_->set(GRB_IntParam_LogToConsole, gpt::grb_log_to_console);
  }
  catch (GRBException const& e) {
    std::cout << "GRBException caught while building GRBEnv. Error code = "
              << e.getErrorCode()
              << std::endl
              << e.getMessage() << std::endl
              << "Translation: " << errorCodeTranslation(e.getErrorCode())
              << "\nFor now quitting..." << std::endl;
    exit(201);
  }
}


GRBModelSPtr newModel() {
  if (!env_) initEnvironment();
  GRBModelSPtr model;
  try {
    model.reset(new GRBModel(*env_));
  }
  catch (GRBException const& e) {
    std::cout << "GRBException caught while building new GRBModel. Error code = "
              << e.getErrorCode()
              << std::endl
              << e.getMessage() << std::endl
              << "Translation: " << errorCodeTranslation(e.getErrorCode())
              << "\nFor now quitting..." << std::endl;
    exit(202);
  }
  return model;
}


std::string errorCodeTranslation(int error_code) {
  switch (error_code) {
    case 1:
      return "Model is loaded, but no solution information is available.";
    case 2:
      return "Model was solved to optimality (subject to tolerances), and an optimal solution is available.";
    case 3:
      return "Model was proven to be infeasible.";
    case 4:
      return "Model was proven to be either infeasible or unbounded. To obtain a more definitive conclusion, set the DualReductions parameter to 0 and reoptimize.";
    case 5:
      return "Model was proven to be unbounded. Important note: an unbounded status indicates the presence of an unbounded ray that allows the objective to improve without limit. It says nothing about whether the model has a feasible solution. If you require information on feasibility, you should set the objective to zero and reoptimize.";
    case 6:
      return "Optimal objective for model was proven to be worse than the value specified in the Cutoff parameter. No solution information is available.";
    case 7:
      return "Optimization terminated because the total number of simplex iterations performed exceeded the value specified in the IterationLimit parameter, or because the total number of barrier iterations exceeded the value specified in the BarIterLimit parameter.";
    case 8:
      return "Optimization terminated because the total number of branch-and-cut nodes explored exceeded the value specified in the NodeLimit parameter.";
    case 9:
      return "Optimization terminated because the time expended exceeded the value specified in the TimeLimit parameter.";
    case 10:
      return "Optimization terminated because the number of solutions found reached the value specified in the SolutionLimit parameter.";
    case 11:
      return "Optimization was terminated by the user.";
    case 12:
      return "Optimization was terminated due to unrecoverable numerical difficulties.";
    case 13:
      return "Unable to satisfy optimality tolerances; a sub-optimal solution is available.";
    case 14:
      return "An asynchronous optimization call was made, but the associated optimization run is not yet complete.";
    default:
      return "UNKOWN RETURN CODE";
  }
}

};

#endif