#include <iostream>
#include <stdlib.h>
#include <cassert>

#include "det_pddl_builder.h"

#include "../mgpt/problems.h"

/******************************************************************************
 *
 * Domain determinization methods
 *
 ******************************************************************************/
DetPDDL buildDeteterministicPDDLFiles(DeterminizationType det_type, state_t const* s)
{
  std::cout << "[" << __FUNCTION__ << "] using '" << gpt::tmp_dir
            << "' as TMP DIR. Use option -b to change it\n";
  std::string template_str1 = std::string(gpt::tmp_dir) +
                              "/buildDeteterministicPDDLFiles_XXXXXX";
  char* template_str2 = strdup(template_str1.c_str());
  char* tmp_dir = mkdtemp(template_str2);
  assert(tmp_dir != nullptr);

  DetPDDL rv;
  rv.tmp_dir = std::string(tmp_dir) + "/";
  free(template_str2);

  std::string det_type_str = "UNKNOWN DETERMINISATION";
  switch (det_type) {
    case MOST_LIKELY_OUTCOMES:
      det_type_str = "Most Likely Outcomes"; break;
    case ALL_OUTCOMES:
      det_type_str = "All Outcomes"; break;
    case ALL_MOST_LIKELY_OUTCOMES:
      det_type_str = "All Most Likely Outcomes"; break;
  }

  rv.domain_file_path = rv.tmp_dir + "domain.pddl";
  std::cout << "[" << __FUNCTION__ << "] " << det_type_str << " det domain: "
            << rv.domain_file_path << std::endl;
  std::ofstream domain;
  domain.open(rv.domain_file_path.c_str());
  DIE(domain.is_open(), "ERROR opening file", 99);
  rv.action_info = printDomainDeterminization(domain, det_type);
  domain.close();

  if (s != nullptr) {
    // Generating the file with the problem
    rv.problem_file_path = rv.tmp_dir + "problem.pddl";
    std::cout << "[" << __FUNCTION__ << "] " << det_type_str << " det problem: "
              << rv.problem_file_path << std::endl;
    std::ofstream problem_file;
    problem_file.open(rv.problem_file_path.c_str());
    DIE(problem_file.is_open(), "ERROR opening file", 99);
    printProblem(problem_file, *s);
    problem_file.close();
  }
  else {
    rv.problem_file_path = "";
  }

  return rv;
}

/*
 * printActionSchemaHeaderAsPPDDL
 */
void printActionSchemaHeaderAsPPDDL(std::ostream& os, ActionSchema const& action_schema,
    std::string const& name)
{
  problem_t const& p = *gpt::problem;
  os << "  (:action " << name  << std::endl;

  if (action_schema.arity() > 0) {
    os << "   :parameters (";
    for (size_t vi = 0; vi < action_schema.arity(); ++vi) {
      os << " ";
      p.domain().terms().print_term(os, action_schema.parameter(vi));
      os << " - ";
      p.domain().types().print_type(os,
          p.domain().terms().type(action_schema.parameter(vi)));
    }
    os << ")" << std::endl;
  }
  os << "   :precondition ";
  action_schema.precondition().print(os, p.domain().predicates(),
      p.domain().functions(), p.domain().terms());
  os << std::endl;
}


/*
 * printDomainDeterminization
 */
DetActionNameToInfo printDomainDeterminization(std::ostream& os,
    DeterminizationType det_type)
{
  problem_t const& p = *gpt::problem;
  Domain const& d = p.domain();

//  std::cout << p << std::endl << std::endl << std::endl;

  /*** DEFINE ***/
  os << "(define (domain " << d.name() << ")" << std::endl;

  /*** REQUIREMENTS ***/
//  std::string req_str = d.requirements.asPDDL(true, true);
//  if (req_str != "")
//    os << "  " << req_str << std::endl;
  os << "  (:requirements :adl)" << std::endl;

  /*** TYPES ***/
  os << "  (:types";

  // ASSUMPTION(fwt): I'm assuming that the first type is always
  // object (the base type in the pddl that doesn't need to be declared)
  for (Type i = d.types().first_type() + 1;
      i <= d.types().last_type(); ++i)
  {
    os << " ";
    d.types().print_type(os, i);
    // subtypes... Ignoring for now
    for (Type j = d.types().first_type() + 1; j <= d.types().last_type(); ++j) {
      DIE(i == j || ! d.types().subtype(i, j), "Subtypes detected on printAsPPDDL", 171);
    }
  }
  os << ")" << std::endl; //types


  /*** CONSTANTS ***/
  if (d.terms().first_object() <= d.terms().last_object()) {
    os << "  (:constants";
    for (Object i = d.terms().first_object();
        i <= d.terms().last_object(); ++i)
    {
      os << "  ";
      d.terms().print_term(os, i);
      os << " - ";
      d.types().print_type(os, d.terms().type(i));
      if (d.terms().first_object() < d.terms().last_object())
        os << std::endl;
    }
    os << ")" << std::endl; // constants
  }

  /*** PREDICATES ***/
  os << "  (:predicates " << std::endl;
  for (Predicate i = d.predicates().first_predicate();
      i <= d.predicates().last_predicate(); ++i)
  {
    os << "    (";
    d.predicates().print_predicate(os, i);
    size_t arity = d.predicates().arity(i);
    for (size_t j = 0; j < arity; ++j) {
      os << " ?v" << j << " - ";
      d.types().print_type(os, d.predicates().parameter(i, j));
    }
    os << ")" << std::endl;
//    if( d.predicates().static_predicate( i ) )
//      os << " <static>";
  }
  os << "  )" << std::endl; // predicates


  /*** FUNCTIONS ***/
  // Ignoring them..
  for (Function i = d.functions().first_function();
      i <= d.functions().last_function(); ++i)
  {
    if (d.functions().name(i) != "reward") {
      std::cout << "[" << __FUNCTION__ << "] Ignoring function '"
                << d.functions().name(i) << "' from the original problem\n";
    }
  }

  /*** ACTIONS ***/
  DetActionNameToInfo rv;
  for (ActionSchemaMap::const_iterator ai = d.actions().begin();
      ai != d.actions().end(); ++ai)
  {
    Effect const& flat_eff = ai->second->effect().flatten();

    assert(dynamic_cast<QuantifiedEffect const*>(&flat_eff) == nullptr);

    auto const* flat_p_eff = dynamic_cast<ProbabilisticEffect const*>(&flat_eff);
    if (flat_p_eff == nullptr) {
      // This is not a probabilistic action
      printActionSchemaHeaderAsPPDDL(os, *(ai->second), ai->second->name());
      os << "   :effect ";
      flat_eff.print(os, d.predicates(), d.functions(), d.terms(), true);
      os << std::endl;
      os << "  )" << std::endl; // action
      Effect::register_use(&flat_eff);

      rv[ai->second->name()] = DetActionInfo(1, &flat_eff, ai->second);

    } else {
      // PROBABILISTIC ACTION!
      std::set<size_t> most_likely_effs;
      // Finding the set of most likely effects
      if (det_type == ALL_MOST_LIKELY_OUTCOMES) {
        Rational max_p = 0;
        Rational p_noop = 1; // probability that nothing happens
        for (size_t ei = 0; ei < flat_p_eff->size(); ++ei) {
          Rational p_i = flat_p_eff->probability(ei);
          p_noop = p_noop - p_i;
          if (p_i > max_p) {
            most_likely_effs.clear();
            most_likely_effs.insert(ei);
            max_p = p_i;
          } else if (p_i == max_p) {
            most_likely_effs.insert(ei);
          }
        }
        if (p_noop > max_p) {
          // The NO-OP effect has more probability than any other effect,
          // therefore this action will be ignored
          most_likely_effs.clear();
        }
      }

      // Finding the set of most likely effects
      int jsch_mlo_picked_eff = -1;
      if (det_type == MOST_LIKELY_OUTCOMES) {
        std::vector<size_t> most_likely_effs_v;
        Rational max_p = 0;
        Rational p_noop = 1; // probability that nothing happens
        for (size_t ei = 0; ei < flat_p_eff->size(); ++ei) {
          Rational p_i = flat_p_eff->probability(ei);
          p_noop = p_noop - p_i;
          if (p_i > max_p) {
            most_likely_effs_v.clear();
            most_likely_effs_v.emplace_back(ei);
            max_p = p_i;
          } else if (p_i == max_p) {
            most_likely_effs_v.emplace_back(ei);
          }
        }
        if (p_noop > max_p) {
          // The NO-OP effect has more probability than any other effect,
          // therefore this action will be ignored
          most_likely_effs_v.clear();
        } else if (p_noop == max_p) {
          // must break ties between most probable effects and NO-OP
          //
          // note: we treat an index value of most_likely_effs_v.size() as a token that tells us to
          //       use the NO-OP
          auto const idx = rand0toN_l(most_likely_effs_v.size() + 1);
          if (idx == most_likely_effs_v.size()) {
            jsch_mlo_picked_eff = -1;
          } else {
            jsch_mlo_picked_eff = most_likely_effs_v.at(idx);
          }
        } else {
          // break ties between most probable effects
          auto const idx = rand0toN_l(most_likely_effs_v.size());
          jsch_mlo_picked_eff = most_likely_effs_v.at(idx);
        }
        std::cout << "[det_pddl_builder.cc:printDomainDeterminization] most likely effect for "
                  << ai->second->name() << ": " << jsch_mlo_picked_eff << std::endl;
      }

      for (size_t ei = 0; ei < flat_p_eff->size(); ++ei) {

        if (det_type == ALL_MOST_LIKELY_OUTCOMES &&
            most_likely_effs.find(ei) == most_likely_effs.end())
        {
          // Ignoring this effect if the most likely outcome determinization
          // was chosen.
          continue;
        }

        // HACK(jsch): casting ei to int here to avoid compiler warning --- picked_eff needs to be
        // int so we use -1 to represent NOOP. (Can rewrite it with e.g. an std::optional instead.)
        if (det_type == MOST_LIKELY_OUTCOMES && int(ei) != jsch_mlo_picked_eff) {
          // Ignore this effect unless it is the single effect chosen by most likely outcomes det.
          continue;
        }

        std::ostringstream name_os, effect_os;

        // Printing to ostringstream to check if it's not empty
        flat_p_eff->effect(ei).print(effect_os, d.predicates(), d.functions(),
                                     d.terms(), true);
        std::string effect_str = effect_os.str();
        if (effect_str.find("(") != std::string::npos) {
          name_os << ai->second->name() << "-" << (ei+1);
          std::string name = name_os.str();
          printActionSchemaHeaderAsPPDDL(os, *(ai->second), name);
          os << "   :effect " << effect_str << std::endl;
          os << "  )" << std::endl; // action

          Effect::register_use(&flat_p_eff->effect(ei));
          rv[name] =
            DetActionInfo(flat_p_eff->probability(ei), &flat_p_eff->effect(ei),
                          ai->second);
        }
      }
    }
  }

  os << ")" << std::endl;  // end of domain definition
  return rv;
}


/*
 * printProblem
 */
void printProblem(std::ostream& os, state_t const& initial_state,
                  std::vector<state_t> const* extra_goals)
{
  problem_t const& p = *gpt::problem;
  os << "(define (problem " << p.name() << ")" << std::endl;
  os << "  (:domain " << p.domain().name() << ")" << std::endl;

  /*** OBJECTS ***/
  if (p.terms().first_object() <= p.terms().last_object()) {
    os << "  (:objects";
    for (Object i = p.terms().first_object();
        i <= p.terms().last_object(); ++i)
    {
      os << "  ";
      p.terms().print_term(os, i);
      os << " - ";
      p.domain().types().print_type(os, p.terms().type(i));
      if (p.terms().first_object() < p.terms().last_object())
        os << std::endl;
    }
    os << ")" << std::endl;
  }

  /*** INITIAL STATE ***/
  os << "  (:init";
  // Static predicates
  for (AtomSet::const_iterator ai = p.init_atoms().begin();
      ai != p.init_atoms().end(); ++ai)
  {
    if (p.domain().predicates().static_predicate((*ai)->predicate())) {
      os << " ";
      (*ai)->print(os, p.domain().predicates(), p.domain().functions(),
                   p.terms());
    }
  }
  // Non-static predicates
  initial_state.full_print(os, gpt::problem, false, false);
  os << ")" << std::endl; // initial state

  /*** GOAL STATE ***/
  if (extra_goals == nullptr or extra_goals->empty()) {
    // Base goal states w/o extras

    os << "  (:goal ";
    p.original_goal().print(os, p.domain().predicates(), p.domain().functions(), p.terms());
    os << ")" << std::endl;

    os << ")" << std::endl; // define

  } else{
    // Disjunctive goals state with all the extra goals

// #if not defined NDEBUG
//     std::cout << "[printProblem] adding " << extra_goals->size() << " extra goals" << std::endl;
// #endif

    os << "  (:goal ";
    os << "\n(or\n";

    for (auto const& g : *extra_goals) {
      os << "        (and ";
      os << g.toStringFullWithNegations(&p, false, false);
      os << ")" << std::endl;

    }

    // os << " ( ";
    os << "        ";
    p.original_goal().print(os, p.domain().predicates(), p.domain().functions(), p.terms());
    os << "\n)" << std::endl; // or

    os << ")"; // goal
    os << ")" << std::endl; // define
  }
}
