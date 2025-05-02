#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <memory>
#include <stack>
#include <set>
#include <fstream>

#include "../../utils/die.h"
#include "../../utils/exceptions.h"
#include "../../utils/json_reporter.h"
#include "rational.h"

#ifndef UCHAR_MAX
#define UCHAR_MAX       255
#endif
#ifndef USHORT_MAX
#define USHORT_MAX      65535
#endif

#define NUMBER_NAME     "number"
#define OBJECT_NAME     "object"

#define GPTMAX(x,y)        ((x)>(y)?(x):(y))
#define GPTMIN(x,y)        ((x)<(y)?(x):(y))

// From boost library. The wierd "double" call is really necessary:
// http://www.iar.com/Global/Resources/Developers_Toolbox/C_Cplusplus_Programming/Tips%20and%20tricks%20using%20the%20preprocessor%20%28part%20two%29.pdf
#define STRINGIFY(s) XSTRINGIFY(s)
#define XSTRINGIFY(s) #s

#ifndef MAX_TRACE_SIZE
#define MAX_TRACE_SIZE 1000000
#endif

#define RESET_COLOR "\e[m"
#define BRIGHT_GREEN "\e[32m"
#define BRIGHT_RED "\e[31m"
#define YELLOW "\e[33m"
#define BRIGHT_PURPLE "\e[35m"
#define BRIGHT_CYAN "\e[36m"

// In the cost vector, the first index is the action cost, i.e., the function
// that is generally minimized and the only function in the case of an SSP
#define ACTION_COST 0

// FWT: using the current implementation of state_t and problem, the maximum
// number of atoms of a problem is USHORT_MAX
#if not defined MAX_ATOMS
#define MAX_ATOMS 8192
#endif

#include <bitset>
// using AtomBitset = std::bitset<MAX_ATOMS>;

#include <boost/dynamic_bitset.hpp>
using AtomBitset = boost::dynamic_bitset<>;


// FWT: gcc only
#define DEPRECATED __attribute__((deprecated))

class heuristic_t;
class problem_t;
class Simulator;
class PlannerFFReplan;

class HackedPrSasProblem;
class NonConditionalPrSasProblem;

enum DeterminizationType {MOST_LIKELY_OUTCOMES = 0, ALL_OUTCOMES, ALL_MOST_LIKELY_OUTCOMES };

class SignalManager {
 public:
  SignalManager();
  ~SignalManager() { }
 private:
  static void signalHandler(int signum);
};

#define EXIT(msg) { __EXIT(msg, __FILE__, __LINE__); }
void __EXIT(std::string const& msg, const char* f, int l);


enum StopCriterion {NONE = 0,    // The stop criterion is not defined yet
                    NUM_ROUNDS,  // Stop after a given number of rounds
                    CONV_S0,     // Stop when V(s0) has epsilon-converged
                    CONV_COST    // Stop when the average cost has epsilon-converged (?)
                  };

class Deadline {
 public:
  Deadline() { }
  virtual ~Deadline() { }
  // Returns true if the deadline is over and everything else should stop.
  virtual bool isOver() const = 0;
  // Returns a string explaning why the deadline is over
  virtual std::string const explanation() const = 0;
  // This returns true if there is a time deadline. In this case, remaining is
  // populated with the remaining time in usecs.
  virtual bool remainingTimeInUsec(uint64_t& remaining) const {
    return false;
  }
};


namespace gpt
{
  // to speed up testing
  extern double expected_v_s0;

  // For ICAPS23 to CG-iLAO against other methods
  extern size_t total_computed_qvalues;

  extern bool use_deadend_sink;

  extern std::string cmd;
  extern JsonReporter json_output;
  extern std::string json_output_file;

  extern std::string translate_py_path;

  /***************************************************************************/
  // Name of the ppddl file read as input. This is used by
  // problem_t::parseConstraints
  extern std::string __ppddl_domain_filename;
  extern std::string __ppddl_problem_filename;

  // Caching problem transformations
  extern problem_t const* __strong_relaxation;
  extern std::shared_ptr<HackedPrSasProblem> cached_cond_sas_problem;
  extern std::shared_ptr<NonConditionalPrSasProblem> cached_non_cond_sas_problem;
  /***************************************************************************/


  /***************************************************************************/
  /*                         GUROBI GLOBAL OPTIONS                           */
  extern int    grb_method;
  extern double grb_feasibility_tol;
  extern int    grb_log_to_console;
  extern int    grb_nthreads;
  extern int    grb_numerical_focus;
  /***************************************************************************/

  extern bool use_s3p;
  extern std::string s3p_maxprob_functor;

  extern bool use_h_maxprob_in_phase2;
  extern std::string maxprob_heuristic;

  extern bool idual_use_instability_sink;

  extern uint64_t parsing_cpu_time;
  extern bool ignore_constraints;
  extern bool print_lp;
  extern size_t idual_expansion_per_ite;
  extern double min_prob_reach_goal;
#ifdef USE_CACHE_PROB_OP_ADDS_ATOM
  extern size_t total_saved_prob_op_adds_atom_calls;
  extern size_t total_prob_op_adds_atom_calls;
#endif
  extern bool randomize_actionsT_order;
  extern double given_value_for_vStar_s0;
  extern uint64_t max_cpu_sys_time_usec;
  extern uint64_t max_rss_kb;
  extern Deadline* _deadline_;
  extern StopCriterion stopCriterion;
  extern SignalManager signalManager;
  extern std::string ff_path;
  extern std::string lama_path;
  extern std::string legend_file;
  extern std::set<std::string> debug_signals;
  extern double ignore_effects_with_prob_less_than;
  extern std::shared_ptr<PlannerFFReplan> followable_ffreplan;
  extern std::string followable_ffreplan_param;
  extern bool external_ff_ignore_forall_w_prob_effects;
  extern bool suppress_round_info;
  extern bool default_hp;
  extern std::string algorithm;
  extern bool domain_analysis;
  extern Rational dead_end_value;
  extern unsigned cutoff;
  extern double epsilon;
  extern bool hash_all;
  extern std::string heuristic;
  extern size_t initial_hash_size;
  extern unsigned max_database_size;
  extern bool noise;
  extern double noise_level;
  extern unsigned seed;
  extern int simulations;
  extern unsigned verbosity;
  extern unsigned warning_level;
  extern double heuristic_weight;
  extern size_t xtra;
  extern std::shared_ptr<heuristic_t> heur_ptr;

  extern int max_time_in_secs;

  extern uint64_t start_time;
  extern bool print_turn_details;
  extern std::string execution_simulator;
  extern uint32_t total_execution_rounds;
  extern bool run_to_convergence;

  /* replanner_threshold_t:
   *    The replanner will search for (replanner_threshold_t)-closed policies.*/
  extern size_t replanner_threshold_t; // -T

  extern problem_t const* problem;
  extern Simulator const* simulator;

  extern bool show_applied_policy;
  extern bool show_computed_policy;
  extern uint32_t max_turn;

  // This variable holds the amount of shift applied to the cost of actions in
  // order to make all the actions to have cost > 0
  extern Rational cost_shift;

  // Enables the usage of cost of actions
  extern bool use_action_cost;
  // Enables the usage of cost of states (states and goal rewards). Only works
  // if the cost of actions is enabled
  extern bool use_state_cost;
  // Enables the normalization of the actions cost. That is, all the actions
  // cost will be guaranted to be >= 0
  extern bool normalize_action_cost;
  // Enables the dynamic choice of dead-end value. Since this guarantee is
  // probabilistic, we need an epsilon
  extern bool dynamic_deadend_value;
  extern double dynamic_deadend_epsilon;
  extern std::string tmp_dir;

  // Output file for online experiments (jsch)
  extern std::string online_test_csv_filepath;

  // Interval between evaluations in online experiment
  extern uint64_t online_test_interval_usecs;

  bool setDeadline(Deadline* deadline);
  void removeDeadline();
  void checkDeadline();  // Throws DeadlineReached
  void incCounterAndCheckDeadlineEvery(size_t& counter, size_t mod);
};

inline bool gpt::setDeadline(Deadline* deadline) {
  if (_deadline_)
    return false;
  else {
    _deadline_ = deadline;
    return true;
  }
}

inline void gpt::removeDeadline() { gpt::_deadline_ = 0; }

inline void gpt::checkDeadline() {
  if (_deadline_ && _deadline_->isOver())
    throw DeadlineReachedException(_deadline_->explanation());
}

inline void gpt::incCounterAndCheckDeadlineEvery(size_t& counter, size_t mod) {
  counter++;
  if (counter % mod == 0) {
    gpt::checkDeadline();
  }
}


typedef unsigned char uchar_t;
typedef unsigned short ushort_t;
typedef ushort_t atom_t;

inline void notify(void *ptr, std::string const& name) {
#ifdef MEM_DEBUG
  fprintf( stderr, "notify %s %p\n", name, ptr );
#endif
}

/*
 * readArguments: read the arguments from the command line and set the
 * appropriated flags in the scope gpt::
 */
bool readArguments(int& argc, char**& argv, std::vector<char*>& remaining_args);

/*
 * printBanner: forward declaration. It's not defined in the global.cc because
 * it uses constants defined at compile time.
 */
void printBanner(std::ostream& os);

/*
 * printUsageGlobal: display the flags used for setting the variables in
 * the scope gpt::
 */
void printUsageGlobal(std::ostream& os);

/*
 * printUsageLocal: display the flags of a given binary. This method should be
 * implemented for each file containing a main()
 */
void printUsageLocal(std::ostream& os);

bool readPDDLFile(const char* name);
inline bool readPPDDLDomainFile(const char* name) {
  gpt::__ppddl_domain_filename = std::string(name);
  return readPDDLFile(name);
}

inline bool readPPDDLProblemFile(const char* name) {
  gpt::__ppddl_problem_filename = std::string(name);
  return readPDDLFile(name);
}
#endif // GLOBAL_H
