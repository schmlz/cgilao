#ifndef UTILS_LP_SOLVER_WRAPPER_H
#define UTILS_LP_SOLVER_WRAPPER_H

#if defined USE_CPLEX
#include <ilcplex/ilocplex.h>
#endif

#include "../ext/mgpt/atom_states.h"  // to be able to define the hashes
#include "../ext/mgpt/problems.h"

#if defined USE_GUROBI
#include "gurobi.h"  // Felipe's wrapper
#include "gurobi_c++.h"
#endif

enum class SolveStatus { ERROR, OPTIMAL, INFEASIBLE };

// USAGE NOTES:
// * IMPORTANT: make sure to initialise rows with .row() - otherwise the cplex
//   implementation will cause SEFGAULTS!
// * Currently constraints are implemented with IloRange which means constraints
//   *must* be of the form lambda_1 x_1 + ... + lambda_k x_k == 0 or >= 5 etc.
//   i.e. the RHS must be a constant. If you don't do this, compiler will
//   complain about converting IloConstraint to IloRange.

// IMPLEMENTATION NOTES:
// * this wrapper assumes that the user always uses continuous minimisation
//   problems, so if you want do use MIPs or maximisation that will have to be
//   added
// * TODO currently we have a separate instance of IloEnv for each
//   LPWrapper<LPSolver::CPLEX> - should check whether IloEnv is supposed to be
//   used like gurobi's environment, or whether the idea really is to create a
//   new one for each model.
// * TODO we've implemented constraints with IloRange - the reason for this is
//   that IloRange plays nicer with IloColumn, but there's probably a nicer way
//   to do this. Note: IloRange is subclass of IloConstraint.

namespace LPWrappers
{
const double infinity = std::numeric_limits<double>::infinity();

enum class LPSolver { GUROBI, CPLEX };

// Generic solver template
template <LPSolver S>
class LPWrapper
{
};

#if defined USE_GUROBI

// Gurobi template specialisation
template <>
class LPWrapper<LPSolver::GUROBI>
{
 public:
  LPWrapper() : model_ptr_{Gurobi::newModel()}, model_{*model_ptr_} {}
  ~LPWrapper() {}

  // Renaming types for generality

  using Var    = GRBVar;
  using Row    = GRBLinExpr;
  using Column = GRBColumn;
  using Constr = GRBConstr;

  // Useful hashmaps

  using HashActionPtrGVar          = std::unordered_map<action_t const*, GRBVar>;
  using HashStateHashActionPtrGVar = std::unordered_map<state_t, HashActionPtrGVar, hashState>;
  using HashStateLinExpr           = std::unordered_map<state_t, GRBLinExpr, hashState>;

  HashActionPtrGVar getHashActionPtrGVar() { return HashActionPtrGVar(); }
  HashStateHashActionPtrGVar getHashStateHashActionPtrGVar()
  {
    return HashStateHashActionPtrGVar();
  }
  HashStateLinExpr getHashStateLinExpr() { return HashStateLinExpr(); }

  void set_time_limit_secs(const int limit)
  {
    model_.getEnv().set(GRB_DoubleParam_TimeLimit, limit);
  }

  void reset_time_limit()
  {
    model_.getEnv().set(GRB_DoubleParam_TimeLimit, std::numeric_limits<double>::infinity());
  }

  GRBVar add_variable(const double lower_bound, const double upper_bound,
                      const double obj_coeff = 0, std::string const& name = "")
  {
    return model_.addVar(lower_bound, upper_bound, obj_coeff, GRB_CONTINUOUS, name);
  }

  GRBVar add_variable(const double lower_bound, const double upper_bound,
                      const double objective_coeff, GRBColumn col, std::string const& name = "")
  {
    return model_.addVar(lower_bound, upper_bound, objective_coeff, GRB_CONTINUOUS, col, name);
  }

  GRBLinExpr row() { return GRBLinExpr(); }

  void add_to_row(GRBLinExpr& row, GRBLinExpr term_to_add) { row += term_to_add; }

  GRBColumn column() { return GRBColumn(); }

  void add_to_column(GRBColumn& column, const double coeff, GRBConstr const& term_to_add)
  {
    column.addTerm(coeff, term_to_add);
  }

  // Example:
  //
  // add_constraint_to_objective(flow_constr_lhs[source] == 1.0, "flow_constraint_source");
  GRBConstr add_constraint(GRBTempConstr const& constraint, std::string const& name = "")
  {
    return model_.addConstr(constraint, name);
  }

  void set_minimisation_objective(GRBLinExpr const& expression)
  {
    model_.setObjective(expression, GRB_MINIMIZE);
  }

  void set_maximisation_objective(GRBLinExpr const& expression)
  {
    model_.setObjective(expression, GRB_MAXIMIZE);
  }

  void update_sign_in_objective(GRBVar& var, const double sign)
  {
    var.set(GRB_DoubleAttr_Obj, sign);
  }

  void update() {
    model_.update();
  }

  void solve()
  {
    try {
      model_.optimize();
    } catch (GRBException const& e) {
      std::cout << "GRBException caught. Error code = " << e.getErrorCode() << std::endl
                << e.getMessage() << std::endl
                << "For now quitting..." << std::endl;
      EXIT("Gurobi exception");
    } catch (DeadlineReachedException const& e) {
      std::cout << "DeadlineReachedException caught. Exception explanation:\n"
                << e.what() << std::endl
                << "FWT: figure out what to do now." << std::endl
                << "For now quitting..." << std::endl;
      EXIT("Deadline Reached");
    } catch (...) {
      std::cout << "Some other gurobi error..." << std::endl << "For now quitting..." << std::endl;
      EXIT("Some other gurobi error...");
    }
  }

  void solve_and_print()
  {
    std::cout << "[lp wrapper]: num. variables = " << model_.get(GRB_IntAttr_NumVars) << std::endl
              << "[lp wrapper]: num. constraints = " << model_.get(GRB_IntAttr_NumConstrs)
              << std::endl;
    solve();
    Gurobi::printSolution(model_);
  }

  bool solved_optimally() const { return model_.get(GRB_IntAttr_Status) == GRB_OPTIMAL; }

  SolveStatus solve_status() const {
    if (model_.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
      return SolveStatus::OPTIMAL;
    } else if (model_.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
      return SolveStatus::INFEASIBLE;
    } else {
      NOT_IMPLEMENTED;
      return SolveStatus::ERROR;
    }
  }

  double get_variable_assignment(GRBVar const& var) const { return var.get(GRB_DoubleAttr_X); }

  double get_constr_dual(GRBConstr const& constr) const { return constr.get(GRB_DoubleAttr_Pi); }

  double get_constr_slack(GRBConstr const& constr) const
  {
    return constr.get(GRB_DoubleAttr_Slack);
  }

  double get_objective() const { return model_.get(GRB_DoubleAttr_ObjVal); }

  int get_no_variables() const { return model_.get(GRB_IntAttr_NumVars); }

  int get_no_constraints() const { return model_.get(GRB_IntAttr_NumConstrs); }

  std::string get_variable_name(GRBVar const& var) const { return var.get(GRB_StringAttr_VarName); }

  void write_model(std::string const& filepath) { model_.write(filepath); }

  void check_return_status() const
  {
    int gurobi_status = model_.get(GRB_IntAttr_Status);
    if (gurobi_status != GRB_OPTIMAL) {
      std::cout << "[lp wrapper]: gurobi did not find the optimal solution. "
                << "grb_intattr_status = " << gurobi_status << std::endl
                << "gurobi's error desc = '" << Gurobi::errorCodeTranslation(gurobi_status) << "'"
                << std::endl;
      if (gurobi_status == GRB_TIME_LIMIT) {
        // assert(gpt::_deadline_);
        // assert(gpt::_deadline_->remainingTimeInUsec(remaining_time_in_usec));
        // std::cout << "remaining_time_in_usec = " << remaining_time_in_usec << std::endl;
        std::cout << "[lp wrapper]: no more time to solve the problem."
                  << "Figure out what to do now." << std::endl
                  << "For now quitting..." << std::endl;
        EXIT("Deadline Reached");
      } else {
        std::cout << "[lp wrapper]: gurobi did not find the optimal solution. "
                  << "GRB_IntAttr_Status = " << gurobi_status << std::endl
                  << "gurobi's error desc = '" << Gurobi::errorCodeTranslation(gurobi_status) << "'"
                  << std::endl
                  << "quitting" << std::endl;
        EXIT("Gurobi did not find opt. soln.");
      }
    }
  }

  void print_slack(GRBConstr const& constr) const
  {
    double slack            = constr.get(GRB_DoubleAttr_Slack);
    std::string constr_name = constr.get(GRB_StringAttr_ConstrName);
    std::cout << "[lp] Slack for " << constr_name << " is: " << slack << std::endl;
  }

 private:
  GRBModelSPtr model_ptr_;
  GRBModel& model_;
};

#endif  // USE_GUROBI

#if defined USE_CPLEX

// CPlex template specialisation
template <>
class LPWrapper<LPSolver::CPLEX>
{
 public:
  LPWrapper() : cplex_env_{}, model_{IloModel(cplex_env_)}, cplex_{IloCplex(model_)}
  {
// #if defined NDEBUG
    // turn off output to terminal
    cplex_.setOut(cplex_env_.getNullStream());
// #endif

    // set # threads to 1
    cplex_.setParam(IloCplex::Param::Threads, 1);

    // fix memory limit (in MB)
    //
    // HACK: 1gb less than max_rss_kb to give remainder of algorithm enough space
    double cplex_mem_mb = (gpt::max_rss_kb / 1'000) - 1'000;
    if (cplex_mem_mb <= 0.0) {
      EXIT("cplex was not given enough memory");
    }
    cplex_.setParam(IloCplex::Param::WorkMem, cplex_mem_mb);

    // // ask CPLEX to be careful with numerical instability
    // cplex_.setParam(IloCplex::Param::Emphasis::Numerical, true);

    // // this is to deal with the issue of cplex finding solutions with
    // // slightly negative give-up values
    // cplex_.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 1e-9);

    // fix seed
    cplex_.setParam(IloCplex::Param::RandomSeed, gpt::seed);

    obj_ = IloMinimize(cplex_env_);
    model_.add(obj_);
  }
  ~LPWrapper() {
    // TODO(jsch): are these calls necessary or does cplex do it automatically?
    // model_.end();
    // cplex_env_.end();
  }

  // Renaming types for generality

  using Var       = IloNumVar;
  using Row       = IloExpr;
  using Column    = IloNumColumn;
  using Constr    = IloRange;
  using NumVarArr = IloNumVarArray;
  using NumArr    = IloNumArray;

  // Useful hashmaps

  using HashActionPtrGVar          = std::unordered_map<action_t const*, IloNumVar>;
  using HashStateHashActionPtrGVar = std::unordered_map<state_t, HashActionPtrGVar, hashState>;
  class HashStateLinExpr
  {
   public:
    HashStateLinExpr(LPWrapper<LPSolver::CPLEX>* solver) : solver_{solver} {};
    IloExpr& operator[](state_t const& key)
    {
      if (map_.find(key) == map_.end()) { map_.emplace(key, solver_->row()); }
      return map_.at(key);
    }

   private:
    LPWrapper<LPSolver::CPLEX>* solver_;
    std::unordered_map<state_t, IloExpr, hashState> map_;
  };

  HashActionPtrGVar getHashActionPtrGVar() { return HashActionPtrGVar(); }
  HashStateHashActionPtrGVar getHashStateHashActionPtrGVar()
  {
    return HashStateHashActionPtrGVar();
  }
  HashStateLinExpr getHashStateLinExpr() { return HashStateLinExpr(this); }

  void set_time_limit_secs(const int limit) { cplex_.setParam(IloCplex::Param::TimeLimit, limit); }

  void reset_time_limit() { cplex_.setParam(IloCplex::Param::TimeLimit, 1e+75); }

  void set_max_simplex_iters(const int max)
  {
    cplex_.setParam(IloCplex::Param::Simplex::Limits::Iterations, max);
  }

  void reset_max_simplex_iters()
  {
    cplex_.setParam(IloCplex::Param::Simplex::Limits::Iterations, 9223372036800000000);
  }

  /* Basic methods */

  // Add a variable.
  //
  // Note: to indicate that a variable has no lower/upper bound, pass -/+
  // std::numeric_limits<double>::infinity()
  //
  // Example:
  //
  // auto x2 = wrapper.add_variable(-10.0,
  //                                std::numeric_limits<double>::infinity(),
  //                                "x2");
  IloNumVar add_variable(const double lower_bound, const double upper_bound,
                         const double obj_coeff = 0, const std::string name = "")
  {
    IloNumVar x(cplex_env_, lower_bound, upper_bound, ILOFLOAT);
    const char* name_as_char_ptr = &name[0];
    x.setName(name_as_char_ptr);
    add_to_objective(x, obj_coeff);
    return x;
  }

  void add_to_objective(IloNumVar const& x, const double obj_coeff)
  {
    auto old_expr = obj_.getExpr();
    auto new_expr = old_expr + obj_coeff * x;
    obj_.setExpr(new_expr);
  }

  void set_variable_obj_cost(IloNumVar const& var, const double obj_coeff)
  {
    obj_.setLinearCoef(var, obj_coeff);
  }

  void set_minimisation_objective(IloExpr expression) { obj_.setExpr(expression); }

  void update_sign_in_objective(IloNumVar const& var, const double sign)
  {
    obj_.setLinearCoef(var, sign);
  }

  IloNumVarArray num_var_array() { return IloNumVarArray(cplex_env_); }

  IloNumArray num_array() { return IloNumArray(cplex_env_); }

  void update_signs_in_objectives(IloNumVarArray const& vars, IloNumArray const& signs)
  {
    obj_.setLinearCoefs(vars, signs);
  }

  IloNumVarArray get_var_array(size_t const size) {
    return IloNumVarArray(cplex_env_, size, 0.0, infinity, ILOFLOAT);
  }

  IloNumArray get_float_array(size_t const size) {
    return IloNumArray(cplex_env_, size);
  }

  IloExpr row() { return IloExpr(cplex_env_); }

  void add_to_row(IloExpr& row, IloExpr const& term_to_add) { row += term_to_add; }

  // Example:
  //
  // solver.add_constraint(flow_constr_lhs[s] + 1 * expected_usage[s][nullptr]
  // == 0);
  //
  // Note: You can't pass e.g. flow_constr_lhs[s] == -1 *
  // expected_usage[s][nullptr] since that's an IloConstr
  IloRange add_constraint(IloRange expression, std::string const& name = "")
  {
    // Note: CPlex doesn't want users messing with "constraints", but rather
    // IloRanges - we return the IloRange that was passed in to fit better to
    // the template.
    model_.add(expression);
    // const char* name_as_char_ptr = &name[0];
    // expression.setName(name_as_char_ptr);
    return expression;
  }

  // Adds an expression onto an existing constraint.
  //
  // Example:
  //
  // auto c1 = wrapper.add_constraint(x1 + x2 >= 0);
  // wrapper.add_expr(c1, x2 * 3.0);
  //
  // will result in x1 + 4x2 >= 0.
  void add_to_constraint(IloRange constraint, IloNumExpr modification)
  {
    IloExpr expr = constraint.getExpr();  // get the expression that underlies the constraint
    expr += modification;                 // add the user's modification onto the expression
    constraint.setExpr(expr);             // update the constraint with the modified expression
  }

  void set_constraint_ub(IloRange& constraint, double const ub) {
    constraint.setUB(ub);
  }

  void set_constraint_lb(IloRange& constraint, double const lb) {
    constraint.setLB(lb);
  }

  // Set the bounds for this constraint, i.e. lb <= expression of constraint <= ub
  //
  // if ub = lb we are imposing the constraint expression of constraint == lb / ub
  void set_constraint_range(IloRange& constraint, double const lb, double const ub)
  {
    constraint.setBounds(lb, ub);
  }

  void set_constraint_rhs(IloRange& constraint, double const rhs)
  {
    set_constraint_range(constraint, rhs, rhs);
  }

  double get_constraint_lb(IloRange const& constraint) const { return constraint.getLB(); }

  double get_constraint_ub(IloRange const& constraint) const { return constraint.getUB(); }

  /* Column methods */

  IloNumColumn column() { return IloNumColumn(cplex_env_); }

  IloNumVar add_variable(const double lower_bound, const double upper_bound, const double obj_coeff,
                         IloNumColumn col, std::string const& name = "")
  {
    col += obj_(obj_coeff);
    IloNumVar x(col, lower_bound, upper_bound);
    const char* name_as_char_ptr = &name[0];
    x.setName(name_as_char_ptr);
    return x;
  }
  void add_to_column(IloNumColumn& col, double coefficient, IloRange const& constraint)
  {
    col += constraint(coefficient);
  }

  /* Solving and getting methods */

  double get_variable_assignment(IloNumVar const& var) const { return cplex_.getValue(var); }

  double get_constr_dual(IloRange const& constr) const { return cplex_.getDual(constr); }

  double get_objective() const { return cplex_.getObjValue(); }

  // TODO(jsch): double check, but update should be unecessary for cplex
  void update() { }

  void solve() { cplex_.solve(); }

  void solve_and_print()
  {
    solve();
    cplex_.out() << "Solution status = " << cplex_.getStatus() << std::endl;
    IloNum objval = cplex_.getObjValue();
    std::cout << "Objective = " << objval << std::endl;
  }

  void write_model(std::string const& filepath) const
  {
    const char* filepath_as_char_ptr = &filepath[0];
    cplex_.exportModel(filepath_as_char_ptr);
  }

  void write_solution(std::string const& filepath) const
  {
    const char* filepath_as_char_ptr = &filepath[0];
    cplex_.writeSolution(filepath_as_char_ptr);
  }

  int get_no_variables() const { return cplex_.getNcols(); }

  int get_no_constraints() const { return cplex_.getNrows(); }

  std::string get_variable_name(IloNumVar const& var) const { return var.getName(); }

  bool solved_optimally() const { return cplex_.getStatus() == IloAlgorithm::Status::Optimal; }

  SolveStatus solve_status() const {
    if (cplex_.getStatus() == IloAlgorithm::Status::Optimal) {
      return SolveStatus::OPTIMAL;
    } else if (cplex_.getStatus() == IloAlgorithm::Status::Infeasible) {
      return SolveStatus::INFEASIBLE;
    } else {
      NOT_IMPLEMENTED;
      return SolveStatus::ERROR;
    }
  }

  void check_return_status() const
  {
    const int cplex_status = cplex_.getStatus();
    if (cplex_status != IloAlgorithm::Status::Optimal) {
      std::cout << "[lp wrapper] cplex has not found the optimal solution." << std::endl
                << "Quitting for now." << std::endl;
    }
  }

 private:
  IloEnv cplex_env_;
  IloModel model_;
  IloCplex cplex_;
  IloObjective obj_;
};

#endif  // USE_CPLEX

}  // namespace LPWrappers

#endif  // UTILS_LP_SOLVER_WRAPPER_H