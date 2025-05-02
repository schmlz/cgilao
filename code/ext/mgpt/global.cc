#include <cerrno>
#include <ctime>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/resource.h>
#include <sys/time.h>
#include <unistd.h>

#include "../../utils/exceptions.h"
#include "global.h"
#include "../../utils/utils.h"


// SignalManager: Registring callback function
SignalManager::SignalManager() {
  signal(SIGUSR1, SignalManager::signalHandler);
  signal(SIGTERM, SignalManager::signalHandler);
}

// static
void SignalManager::signalHandler(int signum) {
  std::cout << "Caught signal " << signum << std::endl;
  static const int usr1 = 10;
  static const int terminate = 15;
  if (signum == usr1 || signum == terminate) {
    std::cout << "Saving JSON Report data and exiting with value " << signum << std::endl;
    gpt::json_output.overwrite("return code", signum);
    gpt::json_output.insert("signal caught with total time", get_cpu_and_sys_time_usec());
    gpt::json_output.insert("signal caught with max rss", get_max_resident_mem_in_kb());
    gpt::json_output.save();
    exit(signum);
  }
}

void __EXIT(std::string const& msg, const char* f, int l) {
  static const int exit_code = 171;
  std::cout << "EXIT called by " << f << ":" << l << " -- "
            << " msg = '" << msg << "' -- saving json reporter and exiting with code "
            << exit_code << std::endl;
  gpt::json_output.overwrite("return code", exit_code);
  gpt::json_output.insert("exit call", std::string(f) + ":" + std::to_string(l));
  gpt::json_output.insert("exit msg", msg);
  gpt::json_output.insert("exit with total time", get_cpu_and_sys_time_usec());
  gpt::json_output.insert("exit with max rss", get_max_resident_mem_in_kb());
  gpt::json_output.save();
  exit(171);
}


namespace gpt
{
  double expected_v_s0 = -1;

  size_t total_computed_qvalues = 0;

  bool use_deadend_sink = true;

  std::string cmd = "";

  JsonReporter json_output;
  std::string json_output_file = "";

  std::string translate_py_path = "./translate.py";

  bool __use_h_elevator_extra = false;

  std::string __ppddl_domain_filename = "";
  std::string __ppddl_problem_filename = "";
  problem_t const* __strong_relaxation = nullptr;
  std::shared_ptr<HackedPrSasProblem> cached_cond_sas_problem;
  std::shared_ptr<NonConditionalPrSasProblem> cached_non_cond_sas_problem;


  /***************************************************************************/
  /*                         GUROBI GLOBAL OPTIONS                           */
  // Default Gurobi value that automatically chooses the a method. See
  // https://www.gurobi.com/documentation/6.5/refman/method.html
  int grb_method = -1;
  // Default Gurobi value. See
  // https://www.gurobi.com/documentation/6.5/refman/feasibilitytol.html
  double grb_feasibility_tol = 1e-6;
  int grb_log_to_console = 0;
  int grb_nthreads = 1;
  // Default Gurobi value. See
  // https://www.gurobi.com/documentation/6.5/refman/numericfocus.html
  int grb_numerical_focus = 0;
  /***************************************************************************/

  /*
   * Default strategy: equivalent to the i-dual as submitted to ICAPS (i.e.,
   * expand all the fringe).
   */


  bool use_s3p = false;
  std::string s3p_maxprob_functor = "";

  bool use_h_maxprob_in_phase2 = false;
  std::string maxprob_heuristic = "always-1";

  bool idual_use_instability_sink = true;

  uint64_t parsing_cpu_time = 0;
  bool ignore_constraints = false;
  bool print_lp = false;
  size_t idual_expansion_per_ite = 0;
  double min_prob_reach_goal = -1.0;
#ifdef USE_CACHE_PROB_OP_ADDS_ATOM
  size_t total_saved_prob_op_adds_atom_calls = 0;
  size_t total_prob_op_adds_atom_calls = 0;
#endif
  bool randomize_actionsT_order = true;
  double given_value_for_vStar_s0 = 0;
  uint64_t max_cpu_sys_time_usec = 0;
  uint64_t max_rss_kb = 0;
  Deadline* _deadline_ = 0;
  StopCriterion stopCriterion = NONE;
  SignalManager signalManager;
  std::string ff_path = "./ff_mod";
  std::string lama_path = "./lama";
  std::string legend_file = "";
  std::set<std::string> debug_signals = parseDebugSignals();
  double ignore_effects_with_prob_less_than = 0.0;
  std::shared_ptr<PlannerFFReplan> followable_ffreplan;
  std::string followable_ffreplan_param = "";
  bool external_ff_ignore_forall_w_prob_effects = false;
  bool suppress_round_info = false;
  bool default_hp = true;
  std::string algorithm = "lrtdp";
  bool domain_analysis = false;
  Rational dead_end_value = Rational(500); //UINT_MAX;
  unsigned bound = 0;
  unsigned cutoff = 0;
  double epsilon = 0.0001;
  bool hash_all = true;
  std::string heuristic = "smartZero";
  size_t initial_hash_size = 204800;
  unsigned max_database_size = 32;
  bool noise = false;
  double noise_level = 0;
  unsigned seed = 0;
  int simulations = 0;
  unsigned verbosity = 0;
  unsigned warning_level = 0;
  double heuristic_weight = 1;
  size_t xtra = 0;
  std::shared_ptr<heuristic_t> heur_ptr;
  bool print_turn_details = false;
  std::string execution_simulator = "local";
  uint32_t total_execution_rounds = 30;
  int max_time_in_secs = 0;
  uint64_t start_time = 0;
  problem_t const* problem = NULL;
  Simulator const* simulator = NULL;
  bool show_applied_policy = false;
  bool show_computed_policy = false;
  uint32_t max_turn = 10000;
  Rational cost_shift = Rational(0);
  std::string online_test_csv_filepath = "";
  uint64_t online_test_interval_usecs = 1'000'000;

  /*
   * The default is to ignore state cost and consider only action costs. To:
   *  - also consider state cost (i.e., action and state), use -C full
   *  - ignore all the costs (both action and state), use -C ignore
   */
  bool use_action_cost = true;
  bool use_state_cost = false;
  bool normalize_action_cost = false;

  bool dynamic_deadend_value = false;
  double dynamic_deadend_epsilon = 0.001;
  std::string tmp_dir = "/tmp/";
};


#if MEM_DEBUG

  void *
operator new( size_t size )
{
  void *result = malloc( size );
  fprintf( stderr, "new %p %d\n", result, size );
  return( result );
}

  void *
operator new[]( size_t size )
{
  void *result = malloc( size );
  fprintf( stderr, "new[] %p %d\n", result, size );
  return( result );
}

  void
operator delete( void *ptr )
{
  if( ptr )
  {
    fprintf( stderr, "del %p\n", ptr );
    free( ptr );
  }
}

  void
operator delete[]( void *ptr )
{
  if( ptr )
  {
    fprintf( stderr, "del[] %p\n", ptr );
    free( ptr &);
  }
}

#endif // MEM_DEBUG


bool readArguments(int& argc, char**& argv, std::vector<char*>& remaining_args)
{
  if (argc == 1) goto usage;
  ++argv;
  --argc;
  while (argc > 0 && argv[0][0] == '-') {
    if (strlen(argv[0]) == 2) {
      // Single letter argument
      switch (argv[0][1]) {
        case 'a':
          gpt::hash_all = (gpt::hash_all?false:true);
          ++argv;
          --argc;
          break;
        case 'A':
          gpt::show_applied_policy = true;
          argv += 1;
          argc -= 1;
          break;
        case 'b':
          gpt::tmp_dir = argv[1];
          argv += 2;
          argc -= 2;
          break;
        case 'c':
          gpt::cutoff = atoi( argv[1] );
          argv += 2;
          argc -= 2;
          break;
        case 'C':
          if (!strncasecmp(argv[1], "ignore", 6)) {
            gpt::use_action_cost = false;
            gpt::use_state_cost = false;
            gpt::normalize_action_cost = false;
          } else if (!strncasecmp(argv[1], "action", 6)) {
            gpt::use_action_cost = true;
            gpt::use_state_cost = false;
            gpt::normalize_action_cost = false;
            if (!strcasecmp(argv[1], "action:normalize"))
              gpt::normalize_action_cost = true;
            else if (strlen(argv[1]) > 6) {
              std::cout << "Option '-C " << argv[1]
                << "' not recognized" << std::endl;
              exit(-1);
            }
          } else if (!strncasecmp(argv[1], "full", 4)) {
            gpt::use_action_cost = true;
            gpt::use_state_cost = true;
            gpt::normalize_action_cost = false;
            if (!strcasecmp(argv[1], "full:normalize"))
              gpt::normalize_action_cost = true;
            else if (strlen(argv[1]) > 4) {
              std::cout << "Option '-C " << argv[1]
                << "' not recognized" << std::endl;
              exit(-1);
            }
          } else {
            std::cout << "The option '-C " << argv[1]
              << "' was not recognized..."
              << std::endl;
            exit(-1);
          }
          argv += 2;
          argc -= 2;
          break;
        case 'd':
          if (argv[1][0] == 'd') {
            gpt::dynamic_deadend_value = true;
            gpt::dynamic_deadend_epsilon = atof(argv[1]+1);
          } else {
            gpt::dead_end_value = (size_t) gpt::heuristic_weight * atoi(argv[1]);
            gpt::dynamic_deadend_value = false;
          }
          argv += 2;
          argc -= 2;
          break;
        case 'D':
          gpt::max_time_in_secs = atoi(argv[1]);

          if (gpt::max_time_in_secs > 0) {
            struct rlimit rl = { (rlim_t) gpt::max_time_in_secs,
                                 (rlim_t) gpt::max_time_in_secs };
            if (setrlimit(RLIMIT_CPU, &rl)) {
              std::cout << "ERROR trying to set the CPU time limit\n";
              std::cerr << "ERROR trying to set the CPU time limit\n";
            } else
              std::cout << "CPU-TIME LIMIT: " << gpt::max_time_in_secs
                << " secs" << std::endl;
          }
          argv += 2;
          argc -= 2;
          break;
        case 'e':
          gpt::epsilon = atof( argv[1] );
          argv += 2;
          argc -= 2;
          break;
        case 'h':
          gpt::default_hp = false;
          //          !strncmp(gpt::heuristic,argv[1],9) && (strlen(argv[1])==19) && !strcmp(&gpt::heuristic[10],&argv[1][10]);
          gpt::heuristic = argv[1];
          argv += 2;
          argc -= 2;
          break;
        case 'i':
          gpt::initial_hash_size = atoi( argv[1] );
          argv += 2;
          argc -= 2;
          break;
        case 'm':
          gpt::max_database_size = atoi( argv[1] );
          argv += 2;
          argc -= 2;
          break;
        case 'M':
          gpt::max_turn = (uint32_t) atoi(argv[1]);
          argv += 2;
          argc -= 2;
          break;
        case 'p':
          gpt::default_hp = false;
          gpt::algorithm = argv[1];
          argv += 2;
          argc -= 2;
          break;
        case 'P':
          gpt::show_computed_policy = true;
          argv += 1;
          argc -= 1;
          break;
        case 'r':
          gpt::seed = atoi(argv[1]);
          argv += 2;
          argc -= 2;
          break;
        case 'R':
          gpt::total_execution_rounds = (uint32_t) atoi(argv[1]);
          DIE(gpt::stopCriterion == NONE,
              "Another stop criterion was specified before", -1);
          gpt::stopCriterion = NUM_ROUNDS;
          argv += 2;
          argc -= 2;
          break;
        case 's':
          gpt::simulations = atoi( argv[1] );
          argv += 2;
          argc -= 2;
          break;
        case 't':
          gpt::print_turn_details = true;
          argv += 1;
          argc -= 1;
          break;
        case 'v':
          gpt::verbosity = atoi( argv[1] );
          argv += 2;
          argc -= 2;
          break;
        case 'w':
          gpt::heuristic_weight = atof(argv[1]);
          gpt::dead_end_value = gpt::dead_end_value * (size_t) gpt::heuristic_weight;
          argv += 2;
          argc -= 2;
          break;
        case 'x':
          gpt::xtra = atoi( argv[1] );
          argv += 2;
          argc -= 2;
          break;
        case 'X':
          gpt::execution_simulator = argv[1];
          argv += 2;
          argc -= 2;
          break;
        case 'z':
          gpt::domain_analysis = (gpt::domain_analysis?false:true);
          ++argv;
          --argc;
          break;
        default:
          std::cout << "Flag not recognized: " << argv[0] << std::endl;
          goto usage;
      }  // end switch
    }  // end if strlen(argv[0]) == 2
    else if (strlen(argv[0]) > 2 && argv[0][1] == '-') {
      // Long option, i.e., --something
      if (!strcasecmp(*argv, "--legend")) {
        gpt::legend_file = std::string(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--ff_path")) {
        gpt::ff_path = std::string(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--lama_path")) {
        gpt::lama_path = std::string(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strncasecmp(*argv, "--follow_ff:", 12)) {
        gpt::followable_ffreplan_param = std::string(argv[0]);
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--suppress_round_info")) {
        gpt::suppress_round_info = true;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--ignore_effects_with_prob_less_than")) {
        gpt::ignore_effects_with_prob_less_than = atof(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--conv_s0")) {
        DIE(gpt::stopCriterion == NONE,
            "Another stop criterion was specified before", -1);
        gpt::stopCriterion = CONV_S0;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--conv_cost")) {
        DIE(gpt::stopCriterion == NONE,
            "Another stop criterion was specified before", -1);
        gpt::given_value_for_vStar_s0 = atof(argv[1]);
        gpt::stopCriterion = CONV_COST;
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--max_rss_kb")) {
        gpt::max_rss_kb = atoi(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--max_cpu_time_sec")) {
        gpt::max_cpu_sys_time_usec = 1000000 * (uint64_t) atoi(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--no_rand_action_ordering")) {
        gpt::randomize_actionsT_order = false;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--min_prob_reach_goal")) {
        gpt::min_prob_reach_goal = atof(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--print_lp")) {
        gpt::print_lp = true;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--expansion_per_ite")) {
        gpt::idual_expansion_per_ite = atoi(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--ignore_constraints")) {
        gpt::ignore_constraints = true;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--no_instability_sink")) {
        gpt::idual_use_instability_sink = false;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--h-max-prob")
                 || !strcasecmp(*argv, "--h-maxprob"))
      {
        gpt::maxprob_heuristic = argv[1];
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--use_h_maxprob_in_phase2")) {
        gpt::use_h_maxprob_in_phase2 = true;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--s3p")) {
        gpt::use_s3p = true;
        gpt::s3p_maxprob_functor = argv[1];
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--grb_nthreads")) {
        gpt::grb_nthreads = atoi(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--grb-log-to-console")) {
        gpt::grb_log_to_console = 1;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--grb-feasibility-tol")) {
        gpt::grb_feasibility_tol = atof(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--grb-method")) {
        gpt::grb_method = atoi(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--grb-numerical-focus")) {
        gpt::grb_numerical_focus = atoi(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--translate_path") || !strcasecmp(*argv, "--translate-path")) {
        gpt::translate_py_path = argv[1];
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--json")) {
        gpt::json_output_file = argv[1];
        gpt::json_output.setFilename(gpt::json_output_file);
        gpt::json_output.save();
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--no-deadend-sink")) {
        gpt::use_deadend_sink = false;
        argv += 1;
        argc -= 1;
      } else if (!strcasecmp(*argv, "--expected-v-s0")) {
        gpt::expected_v_s0 = std::stod(argv[1]);
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--online_test_csv_filepath")) {
        gpt::online_test_csv_filepath = argv[1];
        argv += 2;
        argc -= 2;
      } else if (!strcasecmp(*argv, "--online_test_interval_usecs")) {
        gpt::online_test_interval_usecs = std::stoull(argv[1]);
        argv += 2;
        argc -= 2;
      } else {
        std::cout << "Flag not recognized: " << argv[0] << std::endl;
        goto usage;
      }
    }  // end if strlen(argv[0]) > 2
    else {
      goto usage;
    }
  }

  if (argc == 0 || argc > 2) {
usage:
    printUsageGlobal(std::cout);
    if (argc > 0) {
      std::cerr << "\n\nERROR! Parameter '" << argv[0] << "' not recognized!"
                << " Didn't try the parameters after that one too...\n" << std::endl;
    }
    else {
      std::cerr << "\n\nERROR! Not enough parameters" << std::endl;
    }
    exit(7);
  }
  else {
    for (int i = 0; i < argc; i++) {
      remaining_args.push_back(argv[i]);
    }
    return true;
  }
}

void printUsageGlobal(std::ostream& os) {
  printBanner(os);
  printUsageLocal(os);
  os << std::endl << std::endl
     << "==================================================================================="
     << std::endl
     << "GLOBAL Options:" << std::endl << std::endl
     << "  [-a]                      (toggle hash-all states, default = off)" << std::endl
     << "  [-A]                      (print the policy applied by the planner during simulation)" << std::endl
     << "  [-b <dir>]                (set the temporary directory for working files. Default = " << gpt::tmp_dir << std::endl
     << "  [-C <cost-policy>]        (select how to deal with the cost/reward, see options bellow, default ignore)" << std::endl
     << "  [-c <cutoff>]             (default = <infty>)" << std::endl
     << "  [-d <dead-end-value>]     (default = " << gpt::dead_end_value << ", see options bellow)" << std::endl
     << "  [-D <secs>]               (max CPU time resource for the process. Default 0 = no limit)" << std::endl
     << "  [-e <epsilon>]            (default = " << gpt::epsilon << ")" << std::endl
     << "  [-h <heuristic-stack>]    (default = \"atom-min-1-forward|min-min-lrtp\")" << std::endl
     << "  [-i <initial-hash-size>]  (default = 16536)" << std::endl
     << "  [-m <max-database-size>]  (default = 32)" << std::endl
     << "  [-M <max_turn>]           (max number of actions applicable in each round. Default = "
     << gpt::max_turn << ")" << std::endl
     << "  [-p <planner>]            (default = lrtdp)" << std::endl
     << "  [-P]                      (print the policy OBTAINED by the planner from the initial state)" << std::endl
     << "  [-r <random-seed>]        (default = 0)" << std::endl
     << "  [-R <rounds>]             (default = 30)" << std::endl
     << "  [-s <simulations>]        (default = 0)" << std::endl
     << "  [-t]                      (Turn on the output of each turn detail)" << std::endl
     << "  [-T <t>]                  (replan using t-closed policies)" << std::endl
     << "  [-v <verbose-level>]      (default = 0)" << std::endl
     << "  [-w <heuristic-weight>]   (default = 1)" << std::endl
     << "  [-X <simulator>]          (default = local)" << std::endl
     << "  [-z]                      (toggle domain analysis, default = off)" << std::endl
     << "  --follow_ff:<options>*     use --follow_ff:help to see the real options" << std::endl
     << "  --suppress_round_info" << std::endl
     << "  --ignore_effects_with_prob_less_than P" << std::endl
     << "  <planner>         := random | vi | rtdp | rtdpNoTrial | lrtdp | ldfs | asp | hdp-<n> | ssipp | ffreplan"
     << std::endl
     << "  <heuristic-stack> := <heuristic-stack> '|' <heuristic> | <heuristic>" << std::endl
     << "  <heuristic>       := zero | smartZero | ff | look-<n> | min-min-lrtdp |" << std::endl
     << "                       atom-min-1-forward" << std::endl
     << "  <simulator>       := local" << std::endl
     << "  <cost-policy>     := (ignore|action|full):(<empty>|normalize) " << std::endl
     << "  <dead-end-value>  := <fixed-dead-end-value> | d<dynamic-dead-end-epsilon>" << std::endl
     << std::endl
     << "A stop criterion must be provided, that is, one of the following flags must be passed: " << std::endl
     << "  -R <number of round>" << std::endl
     << "  --conv_cost <cost>   wait until V(s0) epsilon converge to that cost" << std::endl
     << "  --conv_s0            wait the epsilon convergence of V to V*" << std::endl
     << std::endl << std::endl;

  os
    << "MCMP (planner: i-dual-mcmp) options:\n"
    << "  --grb-method 0\t This gubori option seems more num. stable for mcmp\n"
    << "  --h-max-prob <name>\t MaxProb heuristic. Values:\n"
    << "\t always-1               Uninformative heuristics\n"
    << "\t h-max                  1 if h-max(s) doesn't say it is a dead-end. 0 otherwise\n"
    << "\t lookahead-h-max:<n>    look-a-head of <n> over the h-max (above)\n"
    << "\t reuse                  "
    <<        "Uses the heuristic V* by returning 1 if not a dead-end, 0 otherwise\n"
    << "\t                        the advantage (for now...) is that the heuristic will be cached\n"
    << "  --use_h_maxprob_in_phase2\t Uses MaxProb heuristics in MinCost phase. "
    <<      "Since the heuristics are not very informative, it doesn't seem to help much\n\n";
}

char const* current_file;
extern int yyparse();
extern FILE* yyin;

bool readPDDLFile(char const* name) {
  yyin = fopen(name, "r");
  if (yyin == NULL) {
    std::cout << "parser:" << name << ": " << strerror(errno) << std::endl;
    return (false);
  }
  else {
    current_file = name;
    bool success;
    try {
      success = (yyparse () == 0);
    }
    catch (Exception& exception) {
      fclose(yyin);
      std::cout << exception << std::endl;
      return false;
    }
    fclose(yyin);
    return success;
  }
}
