#include <iostream>

#include "ext/mgpt/actions.h"
#include "ext/mgpt/domains.h"
#include "ext/mgpt/global.h"
#include "ext/mgpt/problems.h"
#include "ext/mgpt/states.h"

#include "heuristics/heuristic_factory.h"
#include "heuristics/heuristic_cssp_factory.h"

#include "planners/planner_factory.h"

#include "simulators/simulator.h"

#include "ssps/ppddl_adaptors.h"
#include "ssps/policy.h"

#include "utils/exceptions.h"
#include "utils/utils.h"

#include "utils/banner.h"

#include "utils/utils.h" // for get_cputime_usec()


void printUsageLocal(std::ostream& os) {
  os << "Usage: solver_ssp <option>* "
     << "[<domain-and-problem-file> | <domain-file> <problem-file> [problem-name]]"
     << std::endl << std::endl;
}


int main(int argc, char** argv) {

  gpt::start_time = get_time_usec();

  // Removing buffer from stdout/cout
#ifndef NDEBUG
  setvbuf(stdout, NULL, _IONBF, 0);
#endif

  Planner* planner = 0;
  ushort_t seed[3];

  // Setting the seed as time. If the seed is given as a parameter, the
  // parameter will overwrite this one
  gpt::seed = time(NULL);


  // set command line
  std::ostringstream cmd;
  for( int i = 0; i < argc; ++i )
    cmd << argv[i] << " ";
  gpt::json_output.insert("cmd", cmd.str());

  // read arguments and print banner
  std::vector<char*> remaining_args;
  if (!readArguments(argc, argv, remaining_args))
    return -1;

  printBanner(std::cout);
  std::cout << "**" << std::endl;
  std::cout << "COMMAND: " << cmd.str() << std::endl;
  std::cout << "PLANNER: \"" << gpt::algorithm << "\"" << std::endl;
  std::cout << "HEURISTIC: \"" << gpt::heuristic << "\"" << std::endl;
  std::cout << "STOP CRITERION: ";

  if (gpt::stopCriterion == NONE) {
    std::cout << "Not expecified... Error! See help" << std::endl;
    return -1;
  }
  else if (gpt::stopCriterion == NUM_ROUNDS) {
    std::cout << "Fixed number of rounds (" << gpt::total_execution_rounds
              << ")" << std::endl;
  }
  else if (gpt::stopCriterion == CONV_S0) {
    std::cout << "Epsilon-convergence of V(s0). Epsilon = " << gpt::epsilon
      << std::endl;
  }
  else if (gpt::stopCriterion == CONV_COST) {
    std::cout << "Epsilon-convergence of V*(s0) to the given cost. "
      << "Cost Given = " << gpt::given_value_for_vStar_s0
      << "\tEpsilon = "  << gpt::epsilon << std::endl;
  }

  std::cout << "COST POLICY: ";
  if (gpt::use_action_cost) {
    if (gpt::use_state_cost)
      std::cout << "FULL (actions & states)";
    else
      std::cout << "ACTIONS only";

    if (gpt::normalize_action_cost)
      std::cout << ", NORMALIZED";
    else
      std::cout << ", unnormalized";
  } else
    std::cout << "IGNORE";
  std::cout << std::endl;
  std::cout << "DEAD-END VALUE: ";
  if (gpt::dynamic_deadend_value)
    std::cout << "DYNAMIC, EPSILON = " << gpt::dynamic_deadend_epsilon
              << std::endl;
  else
    std::cout << "STATIC, VALUE = " << gpt::dead_end_value << std::endl;

  std::cout << "SEED: " << gpt::seed << std::endl;
  gpt::json_output.insert("seed", gpt::seed);

  std::cout << "HOSTNAME: " << getHostname() << std::endl;

  if (gpt::ignore_effects_with_prob_less_than > 0.0 &&
      gpt::execution_simulator == "local")
  {
    std::cout << IN_COLOR(BRIGHT_RED,
        "SIMULATION IS CONSIDERING AN IMPRECISE MODEL! "
        "In order to avoid this, use mdpsim to decople planning "
        "model and simulation model") << std::endl;
  }

  // set random seeds
  seed[0] = seed[1] = seed[2] = gpt::seed;
  srand48(gpt::seed);
  seed48(seed);

  // TODO: Add functions to time things before any round starts. The timing
  // here will not work because of this issue.
  assert(remaining_args.size() > 0);
  assert(remaining_args.size() < 3);

  // Remaining parameters semantics by the length of the vector:
  //
  // Length: | 1st position             | 2nd position   | 3rd pos   |
  // --------|--------------------------|----------------|-----------|
  // 1       | domain + prob ppddl file | --             | --        |
  // 2       | domain ppddl file        | prob pddl file | --        |
  // 3       | domain ppddl file        | prob pddl file | prob name |

  // Always parse first argument (either domain+prob or domain ppddl)
  if (remaining_args.size() == 1) {
    if (!readPPDDLDomainFile(remaining_args[0])) {
      std::cout << "[main]: ERROR: couldn't read parse the Domain+Problem file `"
                << remaining_args[0] << "'" << std::endl;
      return -1;
    }
    gpt::__ppddl_problem_filename = gpt::__ppddl_domain_filename;
    gpt::json_output.insert("ppddl input", gpt::__ppddl_domain_filename);
  }
  else {
    //remaining_args.size() > 1
    if (!readPPDDLDomainFile(remaining_args[0])) {
      // There is at least prob pddl file
      std::cout << "[main]: ERROR: couldn't read parse the domain file `"
                << remaining_args[0] << "'" << std::endl;
      return -1;
    }
    if (!readPPDDLProblemFile(remaining_args[1])) {
      // There is at least prob pddl file
      std::cout << "[main]: ERROR: couldn't read parse problem file `"
                << remaining_args[1] << "'" << std::endl;
      return -1;
    }
    gpt::json_output.insert("ppddl input", gpt::__ppddl_domain_filename + " " +
                                            gpt::__ppddl_problem_filename);
  }

  problem_t *problem = NULL;
  if (remaining_args.size() == 3) {
    problem = (problem_t*)problem_t::find(remaining_args[2]);
    if (!problem) {
      std::cout << "[main]: ERROR: problem `" << remaining_args[2]
                << "' is not defined." << std::endl;
      return -1;
    }
  }
  else {
    problem = problem_t::first_problem();
    if (!problem) {
      std::cout << "[main]: ERROR: no problem was defined." << std::endl;
      return -1;
    }
  }

  assert(problem);
  gpt::problem = problem;
  std::cout << "PROBLEM NAME: " << problem->name() << std::endl;
  std::cout << "**" << std::endl;

  if (gpt::verbosity >= 300) {
    std::cout << "[domain-begin]" << std::endl
      << problem->domain() << std::endl
      << "<domain-end>" << std::endl;
  }

  std::cout << "\nCSVHACK"
          << "," << gpt::__ppddl_domain_filename
          << "," << gpt::__ppddl_problem_filename
          << "," << gpt::seed
          << "," << gpt::algorithm
          << "," << gpt::heuristic
          << "," << -1
          << "," << -1
          << "," << -1
          << "," << -1
          << "\n\n";

  //xxxxxx move after timer is started
  // instantiate actions
  try {
//    START_TIMING("instantiating_acts");
    problem->instantiate_actions();
//    STOP_TIMING("instantiating_acts");

//    START_TIMING("flattening");
    problem->flatten();
//    STOP_TIMING("flattening");

//    START_TIMING("prob_initing");
    state_t::initialize(*problem);
//    STOP_TIMING("prob_initing");

    if (gpt::verbosity >= 300) {
      std::cout << "[problem-begin]" << std::endl << "goal: ";
      problem->goalT().print(std::cout);
      std::cout << std::endl << "[problem-end]" << std::endl;
    }
  }
  catch (std::exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }


  if (gpt::verbosity >= 300)
    std::cout << "**" << std::endl;
  std::cout << "[begin-session]" << std::endl;

  // initialize algorithm + heuristic + planner and execute
  try {
    if (problem->constraints().size() > 0) {
      // SSP or solving C-SSP as SSP
      std::cout << "[Solver SSP] This problem has "
                << problem->constraints().size() << " constraints and they will be "
                << "IGNORED. Use solver_cssp to consider the constraints"
                << std::endl;
    }

    SSPfromPPDDL ssp(*problem);
    createGlobalHeuristic(ssp, gpt::heuristic);

    // HACK: if the problem is trivial then algorithms don't output their data.
    //       So, we do it here.
    if (ssp.isGoal(ssp.s0())) {
      std::cout << "===== PROBLEM IS TRIVIAL =====" << std::endl;
      std::cout << "\nCSVHACK"
                << "," << gpt::__ppddl_domain_filename
                << "," << gpt::__ppddl_problem_filename
                << "," << gpt::seed
                << "," << gpt::algorithm
                << "," << gpt::heuristic
                << "," << 0 // v_.value(ssp_.s0())
                << "," << get_cputime_usec()
                << "," << 0 // gpt::heur_ptr->totalCalls()
                << "," << 0 // gpt::total_computed_qvalues
                << "\n\n";
    }

    planner = createPlanner(ssp, gpt::algorithm);

    if (!planner) {
      // If no SSP planner was found, trying a C-SSP solver instead
      ConstrSSPIface* cssp = dynamic_cast<ConstrSSPIface*>(&ssp);
      assert(cssp);
      HeuristicCSSPUniqPtr h_ssp(new MainCostOnlyHeuristic(*cssp, gpt::heur_ptr.get()));
      planner = createPlanner(*cssp, gpt::algorithm, std::move(h_ssp));
    }

    if (!planner) {
      std::cout << "[ERROR] No SSP or C-SSP planner called '" << gpt::algorithm
                << "' was found. Quitting" << std::endl;
      exit(-1);
    }

    problem->no_more_atoms();

    // Setting-up the execution environment. Using C-SSPs to make sure the
    // simulator will output the value of the extra cost functions. Notice that
    // the constraints, if they exists, are not enforced by the simulator (in
    // fact, I don't believe they are enforcible there).
    ConstrSSPfromPPDDL cssp(*problem);
    Simulator* execution_simulator = createSimulator(cssp,
                                                    gpt::execution_simulator);

    if (!execution_simulator)
      return -1;

    gpt::simulator = execution_simulator;

    // Saving the policy of the planner if needed
    if (gpt::show_applied_policy || gpt::show_computed_policy)
      execution_simulator->setSavePolicy(true);


    execution_simulator->setOutputTurns(gpt::print_turn_details);
    if (gpt::stopCriterion == NUM_ROUNDS) {
      SystemResourcesDeadline* eval_deadline = nullptr;

      try {
        if (gpt::max_cpu_sys_time_usec > 0 || gpt::max_rss_kb > 0) {
          eval_deadline = new SystemResourcesDeadline(gpt::max_cpu_sys_time_usec,
                                                      gpt::max_rss_kb);
          gpt::setDeadline(eval_deadline);
        }

        // Restarting the seed. The idea is, if pi_1 and pi_2 are the same
        // but obtained in a different way (and potentially using different
        // numbers of calls to the random generator), they should still be
        // evaluated equally.
        // FWT: not sure if the +1 is really necessary, however, it won't
        // hurt.
        std::cout << "resetting the seed for simulation\n";
        srand48(gpt::seed + 1);
        seed48(seed + 1);
        auto rounds = execution_simulator->simulateNRounds(
            gpt::total_execution_rounds, planner, gpt::max_turn);
        size_t n = rounds[0].accumulatedCost.size();
        std::vector<Rational> avg_cost(n, Rational(0));
        std::vector<size_t> n_round_status(10, 0); // 10 is a hack
        for (auto const& r : rounds) {
          for (size_t i = 0; i < n; ++i) {
            avg_cost[i] += r.accumulatedCost[i];
          }
          n_round_status[(size_t) r.exitStatus]++;
        }
        for (size_t i = 0; i < n; ++i) {
          avg_cost[i] /= (double) gpt::total_execution_rounds;
        }
        FunctionTable const& ft = gpt::problem->domain().functions();
        for (size_t k = 0; k < ft.size(); ++k) {
          std::cout << "Observed Avg ";
          if (ft.name(k) == "reward") {
            std::cout << "cost";
          }
          else {
            std::cout << ft.name(k);
          }
          std::cout << " = " << avg_cost[k] << std::endl;
        }
        for (size_t i = 0; i < n_round_status.size(); ++i) {
          if (n_round_status[i] > 0) {
            EndOfRoundStatus s = (EndOfRoundStatus) i;
            std::cout << "Observed Ratio of " << s << " rounds = "
                      << (n_round_status[i] / (float) gpt::total_execution_rounds)
                      << std::endl;
          }
        }
      }
      catch (DeadlineReachedException& e) {
        std::cout << std::endl
                  << "[" << get_human_readable_timestamp() << "] "
                  << eval_deadline->explanation() << " Aborting..." << std::endl;
        EXIT("Deadline Exception caught");
      }
    }
    else if (gpt::stopCriterion == CONV_S0) {
      HeuristicPlanner* heur_planner = dynamic_cast<HeuristicPlanner*>(planner);
      if (!heur_planner) {
        std::cout << "The chosen planner is not an heuristic planner, "
          << "therefore the convergence over s0 is not possible." << std::endl;
        return -1;
      }
      OptimalPlanner* opt_planner = dynamic_cast<OptimalPlanner*>(planner);
      if (!opt_planner) {
        std::cout << "The chosen planner is not an optimal planner!"
          << "therefore the convergence over s0 implemented for opt planners "
          << "only for now" << std::endl;
        return -1;
      }
      else {
        state_t s0 = problem->get_initial_state();
        SystemResourcesDeadline eval_deadline(gpt::max_cpu_sys_time_usec,
                                              gpt::max_rss_kb);
        try {
          gpt::setDeadline(&eval_deadline);
          opt_planner->optimalSolution();
        }
        catch (DeadlineReachedException& e) {
          std::cout << std::endl
              << "[" << get_human_readable_timestamp() << "] "
              << eval_deadline.explanation() << " Aborting..."  << std::endl;
          std::cout << "V_lb(s0) = " << heur_planner->value(s0) << std::endl;
          EXIT("Deadline Exception caught");
        }
      }

    }
    else if (gpt::stopCriterion == CONV_COST) {
      std::cout << "HACK! CAUTION: we declare that it has converge when "
        << "V*(s0) - V_cur(s0) <= epsilon where V*(s0) is given in the "
        << "command line. This should be used with care!" << std::endl;

      HeuristicPlanner* heur_planner = dynamic_cast<HeuristicPlanner*>(planner);
      if (!heur_planner) {
        std::cout << "The chosen planner is not an heuristic planner, "
          << "therefore the convergence to fixed value of s0 is not possible."
          << std::endl;
        return -1;
      }

      state_t s0 = problem->get_initial_state();
      SystemResourcesDeadline eval_deadline(gpt::max_cpu_sys_time_usec,
                                            gpt::max_rss_kb);
      try {
        gpt::setDeadline(&eval_deadline);
        while (true) {
          execution_simulator->simulateRound(heur_planner, gpt::max_turn);
          if (gpt::given_value_for_vStar_s0 - heur_planner->value(s0) <= gpt::epsilon)
          {
            std::cout << "Convergence reached! V(s0) = "
              << heur_planner->value(s0) << " (error wrt to given value is "
              << (gpt::given_value_for_vStar_s0 - heur_planner->value(s0))
              << ")" << std::endl;
            break;
          }
        }
      }
      catch (DeadlineReachedException& e) {
        std::cout << std::endl
          << "[" << get_human_readable_timestamp() << "] "
          << eval_deadline.explanation() << " Aborting..."  << std::endl;
        std::cout << "V_lb(s0) = " << heur_planner->value(s0) << std::endl;
        EXIT("Deadline Exception caught");
      }
    }

    // Showing the policy of the planner if needed
    HeuristicPlanner* heur_planner = dynamic_cast<HeuristicPlanner*>(planner);
    if (heur_planner) {
      state_t s0 = problem->get_initial_state();
      double val_cur = heur_planner->value(s0);
      std::cout << "V(s0) = " << val_cur << std::endl;
      if (gpt::expected_v_s0 > 0) {
        if (fabs(val_cur - gpt::expected_v_s0) > 0.1) {
          std::cout << "\n\nEXPECT V*(s0) = " << gpt::expected_v_s0
                    << " but the planner returned " << val_cur
                    << " with differs by more than 0.1 ("
                    << fabs(val_cur - gpt::expected_v_s0) << ")\n\n";
          exit(-111);
        }
      }
      gpt::json_output.insert("v_star_s0", val_cur);
    }

    if (gpt::show_applied_policy || gpt::show_computed_policy) {
      DetPolicy pi;
      if (gpt::show_applied_policy)
        pi = execution_simulator->getSavedPolicy();
      else if (gpt::show_computed_policy) {
        NOT_IMPLEMENTED;
      }
      std::cout << "<final-policy>" << std::endl
                << pi << "</final-policy>" << std::endl;
    }
    delete execution_simulator;
  }
  catch (std::exception& e) {
    std::cout << e.what() << std::endl;
    state_t::statistics(std::cout);
    std::cout << "CPU+Sys time: " << get_cpu_and_sys_time_usec() << std::endl;
    std::cout << "Max Resident Mem: " << get_max_resident_mem_in_kb() << " KB" << std::endl;
    std::cout << "[end-session] due to Exception" << std::endl;
    return -1;
  }

  // print statistics and clean
  planner->statistics(std::cout, gpt::verbosity);
  delete planner;

  state_t::statistics( std::cout );
  state_t::finalize();

  uint64_t cpu_sys_time = get_cpu_and_sys_time_usec();
  std::cout << "Max Resident Mem: " << get_max_resident_mem_in_kb() << " KB\n"
            << "Human formatted cputime+sys time: "
            << humanReadableTimeIntervalSince(0)
            << std::endl;

#ifdef USE_CACHE_PROB_OP_ADDS_ATOM
  if (gpt::total_prob_op_adds_atom_calls > 0) {
    std::cout << "Total calls to Prob Op adds atom: "
              << gpt::total_prob_op_adds_atom_calls
              << " -- "
              << gpt::total_saved_prob_op_adds_atom_calls
              << " ["
              << ((float) gpt::total_saved_prob_op_adds_atom_calls /
                          gpt::total_prob_op_adds_atom_calls)
              << "] of them were cached"
              << std::endl;
  }
#endif

  gpt::json_output.insert("total time", cpu_sys_time);
  gpt::json_output.overwrite("return code", 0);

  // return
#ifdef MEM_DEBUG
  std::cerr << "[end-session]" << std::endl;
#endif
  std::cout << "[end-session]" << std::endl;
  return( 0 );
}
