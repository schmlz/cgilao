#ifndef SRC_EXT_MDPSIM_PARSER_DET_PDDL_BUILDER_H__PDB
#define SRC_EXT_MDPSIM_PARSER_DET_PDDL_BUILDER_H__PDB

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "problems.h"
#include "actions.h"
#include "effects.h"
#include "states.h"

namespace PPDDL_PDB {



struct StrEqualIgnoringCase {
  bool operator()(std::string const& s1, std::string const& s2) const {
    return boost::iequals(s1, s2);
  }
};

struct HashStrIgnoringCase  {
  size_t operator()(std::string const& s) const {
    return std::hash<std::string>()(boost::algorithm::to_lower_copy(s));
  };
};

template<class T>
using HashMapStrIgnoringCase = std::unordered_map<std::string, T, HashStrIgnoringCase,
                                                                  StrEqualIgnoringCase>;

enum DeterminizationType {MOST_LIKELY_OUTCOMES = 0, ALL_OUTCOMES};

// Structure to keep track of the probability, effect and original action
// schema of each new deterministic action
class DetActionInfo {
 public:
  double prob;
  Effect const* det_eff;
  ActionSchema const* action_schema;

  DetActionInfo() : prob(-1), det_eff(NULL), action_schema(NULL) { }
  DetActionInfo(Rational q, Effect const* e, ActionSchema const* a)
    : prob(q), det_eff(e), action_schema(a) { }
};


// Hash from strings (ignoring the case) representing the name of the new
// deterministic action (for example, pick-up_1) to DetActionInfo
using DetActionNameToInfo = HashMapStrIgnoringCase<DetActionInfo>;

struct DetPDDL_PDB {
  std::string tmp_dir;
  std::string domain_file_path;
  std::string problem_file_path;
  DetActionNameToInfo action_info;
};


DetPDDL_PDB buildDeteterministicPDDLFiles(Problem const& problem,
    State const& s,
    DeterminizationType det_type,
    std::string const& tmp_dir_prefix);

// Different types of determinization
DetActionNameToInfo printDomainDeterminization(Problem const& problem,
    DeterminizationType det_type,
    std::ostream& os);

// Prints the name, parameters and preconditions of the given action schema
void printActionSchemaHeaderAsPPDDL(
    Problem const& problem,
    ActionSchema const& action_schema,
    std::string const& name,
    std::ostream& os);

// Print the problem, i.e., objects, initial state and goal.
void printProblem(Problem const& problem, State const& s, std::ostream& os);

using ProbAndEffect = std::pair<double, std::string>;
using VecAllOutcome = std::vector<ProbAndEffect>;

VecAllOutcome determinizeActionSchema(ActionSchema const& a);

void determinizeEffect(Effect const* eff, VecAllOutcome& det);

}  // namespace

#endif  // EXTERNAL_DET_PDDL_BUILDER_H
