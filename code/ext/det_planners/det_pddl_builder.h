#ifndef EXTERNAL_DET_PDDL_BUILDER_H
#define EXTERNAL_DET_PDDL_BUILDER_H

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "../mgpt/actions.h"
#include "../mgpt/effects.h"
#include "../mgpt/states.h"

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


// Structure to keep track of the probability, effect and original action
// schema of each new deterministic action
class DetActionInfo {
 public:
  Rational prob;
  Effect const* det_eff;
  ActionSchema const* action_schema;

  DetActionInfo() : prob(-1), det_eff(NULL), action_schema(NULL) { }
  DetActionInfo(Rational q, Effect const* e, ActionSchema const* a)
    : prob(q), det_eff(e), action_schema(a) { }
};


// Hash from strings (ignoring the case) representing the name of the new
// deterministic action (for example, pick-up_1) to DetActionInfo
using DetActionNameToInfo = HashMapStrIgnoringCase<DetActionInfo>;

struct DetPDDL {
  std::string tmp_dir;
  std::string domain_file_path;
  std::string problem_file_path;
  DetActionNameToInfo action_info;
};

// If s is not null, then generate the problem file using s as initial state
DetPDDL buildDeteterministicPDDLFiles(DeterminizationType det_type, state_t const* s);

// Different types of determinization
DetActionNameToInfo printDomainDeterminization(std::ostream& os, DeterminizationType det_type);

// Prints the name, parameters and preconditions of the given action schema
void printActionSchemaHeaderAsPPDDL(std::ostream& os,
    ActionSchema const& action_schema, std::string const& name);

// Print the problem, i.e., objects, initial state and goal.
void printProblem(std::ostream& os, state_t const& initial_state,
                  std::vector<state_t> const* extra_goals = nullptr);

#endif  // EXTERNAL_DET_PDDL_BUILDER_H
