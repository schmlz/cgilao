#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <cassert>

#include "det_pddl_builder.h"
#include "pddl_to_strips.h"

#include "problems.h"

/******************************************************************************
 *
 * Domain determinization methods
 *
 ******************************************************************************/
namespace PPDDL_PDB {

DetPDDL_PDB buildDeteterministicPDDLFiles(Problem const& problem, State const& s,
    DeterminizationType det_type,
    std::string const& tmp_dir_prefix)
{
  std::cout << "[" << __FUNCTION__ << "] using '" << tmp_dir_prefix
            << "' as TMP DIR.\n";

//#define FORCE_DIR

#ifdef FORCE_DIR
  std::string tmp_dir("/tmp/planix_tmp/buildDeteterministicPDDLFiles_E8oyBx/");
#else
  std::string template_str1 = tmp_dir_prefix + "/planix_buildDeteterministicPDDLFiles_XXXXXX";
  char* template_str2 = strdup(template_str1.c_str());
  char* tmp_dir = mkdtemp(template_str2);
  assert(tmp_dir != nullptr);
#endif

  DetPDDL_PDB rv;

  rv.tmp_dir = std::string(tmp_dir) + "/";
#ifndef FORCE_DIR
  free(template_str2);
#endif

  rv.domain_file_path = rv.tmp_dir + "domain.pddl";
  std::cout << "[" << __FUNCTION__ << "] all-outcomes det domain: "
            << rv.domain_file_path << std::endl;
  std::ofstream domain_os;
  domain_os.open(rv.domain_file_path.c_str());
  assert(domain_os.is_open());
  rv.action_info = printDomainDeterminization(problem, det_type, domain_os);
  domain_os.close();

  // Generating the file with the problem
  rv.problem_file_path = rv.tmp_dir + "problem.pddl";
  std::cout << "[" << __FUNCTION__ << "] all-outcomes det problem: "
            << rv.problem_file_path << std::endl;
  std::ofstream problem_os;
  problem_os.open(rv.problem_file_path.c_str());
  assert(problem_os.is_open());
  printProblem(problem, s, problem_os);
  problem_os.close();

  return rv;
}

/*
 * printActionSchemaHeaderAsPPDDL
 */
void printActionSchemaHeaderAsPPDDL(Problem const& p,
    ActionSchema const& action_schema,
    std::string const& name,
    std::ostream& os)
{
  os << "  (:action " << name  << std::endl;

  if (action_schema.arity() > 0) {
    os << "   :parameters (";
    for (auto const& param : action_schema.parameters()) {
      os << " " << param << " - " << p.domain().terms().type(param);
    }
    os << ")" << std::endl;
  }
  os << "   :precondition " << action_schema.precondition();
  os << std::endl;
}


/*
 * printDomainDeterminization
 */
DetActionNameToInfo printDomainDeterminization(Problem const& p, DeterminizationType det_type,
    std::ostream& os)
{
  Domain const& d = p.domain();

  /*** DEFINE ***/
  os << "(define (domain " << d.name() << ")" << std::endl;

  /*** REQUIREMENTS ***/
//  std::string req_str = d.requirements.asPDDL(true, true);
//  if (req_str != "")
//    os << "  " << req_str << std::endl;
  os << "  (:requirements :adl)" << std::endl;

  /*** TYPES ***/
  os << "  (:types";
  os << d.types();
  os << "\n  )" << std::endl; //types


  /*** CONSTANTS ***/
  // TODO(fwt): ignoring it for now since we might not need it.
//  if (d.terms().first_object() <= d.terms().last_object()) {
//    os << "  (:constants";
//    for (Object i = d.terms().first_object();
//        i <= d.terms().last_object(); ++i)
//    {
//      os << "  ";
//      d.terms().print_term(os, i);
//      os << " - ";
//      d.types().print_type(os, d.terms().type(i));
//      if (d.terms().first_object() < d.terms().last_object())
//        os << std::endl;
//    }
//    os << ")" << std::endl; // constants
//  }

  /*** PREDICATES ***/
  os << "  (:predicates ";;
  os << d.predicates();
  os << "\n  )" << std::endl; // predicates


  /*** FUNCTIONS ***/
  // TODO(fwt):  Ignoring objectives and metrics because of the FD translator
//  for (Function i = d.functions().first_function();
//      i <= d.functions().last_function(); ++i)
//  {
//    if (d.functions().name(i) != "reward") {
//      std::cout << "[" << __FUNCTION__ << "] Ignoring function '"
//                << d.functions().name(i) << "' from the original problem\n";
//    }
//  }

  /*** ACTIONS ***/
  DetActionNameToInfo rv;
  std::map<std::string, size_t> metric_to_idx = buildMetricToIdxMap(p);

  for (auto const& it : d.actions()) {

    [[maybe_unused]] std::string const& name = it.first;
    ActionSchema const* action_schema = it.second;

    if (!action_schema) {
      continue;
    }

    VecAllOutcome determinized_effs = determinizeActionSchema(*action_schema);

    for (size_t i = 0; auto const& pr_and_eff : determinized_effs) {
      std::string det_name = action_schema->name() + "-" + std::to_string(i++);

      os << "  (:action " << det_name << "\n"
         << "   :parameters (";
      for (Variable const& var : action_schema->parameters()) {
        Term t = Term(var);
        // TODO: How about variables without a type?
        os << var << " - " << p.domain().terms().type(t) << " ";
      }
      os << ")\n";

      os << "   :precondition " << action_schema->precondition() << "\n"
         << "   :effect " << pr_and_eff.second << "\n"
         << "  )" << std::endl;

      rv[det_name] = DetActionInfo(pr_and_eff.first, nullptr, action_schema);
    }
  }

  os << ")" << std::endl;  // end of domain definition
  return rv;
}


VecAllOutcome determinizeActionSchema(ActionSchema const& a) {
  VecAllOutcome det{{1, ""}};
  determinizeEffect(&a.effect(), det);
  return det;
}


void appendCrossProduct(VecAllOutcome const& in1, VecAllOutcome const& in2, VecAllOutcome& out) {
  for (auto const& i : in1) {
    for (auto const& j : in2) {
      out.push_back({i.first * j.first, i.second + " " + j.second});
    }
  }
}


void determinizeEffect(Effect const* eff, VecAllOutcome& det) {
  assert(eff);
  if (auto const* peff = dynamic_cast<ProbabilisticEffect const*>(eff); peff != nullptr) {
    VecAllOutcome before = det;
    det.clear();
    for (size_t i = 0; i < peff->size(); ++i) {
      VecAllOutcome single_eff{{peff->probability(i), ""}};
      determinizeEffect(&peff->effect(i), single_eff);
      appendCrossProduct(before, single_eff, det);
    }
  }
  else if (auto const* ceff = dynamic_cast<ConjunctiveEffect const*>(eff); ceff != nullptr) {
    VecAllOutcome conj{{1, "(and "}};
    for (auto const& e : ceff->conjuncts()) {
      determinizeEffect(e, conj);
    }
    for (auto& c : conj) {
      c.second += ") ";
    }
    VecAllOutcome before = det;
    det.clear();
    appendCrossProduct(before, conj, det);
  }
  else if (auto const* update = dynamic_cast<UpdateEffect const*>(eff); update != nullptr) {
    // The SAS+ translator does not support the multi-object functions so ignoring it
  }
  // TODO: How about other effects that might not translate correctly?
  else {
    std::ostringstream ost;
    ost << *eff;
    std::string det_eff(ost.str());
    for (auto& pr_and_eff : det) {
      pr_and_eff.second += det_eff + " ";
    }
  }
}

/*
 * printProblem
 */
void printProblem(Problem const& p, State const& s, std::ostream& os) {
  os << "(define (problem " << p.name() << ")" << std::endl;
  os << "  (:domain " << p.domain().name() << ")" << std::endl;

  /*** OBJECTS ***/
  os << "  (:objects" << p.terms() << "\n  )" << std::endl;

  /*** INITIAL STATE ***/
  os << "  (:init";
  // Printing atoms only and ignoring functions, i.e., (= (reward) 0), etc
  for (Atom const* ai : s.atoms) {
    // Not testing for static atoms because we want them in the initial state
    os << " " << *ai;
  }
  os << ")" << std::endl; // initial state

  /*** GOAL STATE ***/
  os << "  (:goal " << p.goal() << ")" << std::endl
     << ")" << std::endl;   // closing define parenthesis
}

}  // namespace
