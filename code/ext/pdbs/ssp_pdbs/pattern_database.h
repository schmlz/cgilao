#ifndef PATTERN_DATABASE_H_SSP_PDB
#define PATTERN_DATABASE_H_SSP_PDB

#include <utility>
#include <vector>

#include "../representations/planning_utils.h"
#include "../representations/sasplus.h"
#include "types.h"

using planning_utils::Cost;
using Variable = SasPlus::SasPlusVariable;

using Problem = SasPlus::SasPlusMOSSP;
using Action = Problem::Action;
using State = Problem::State;
using Value = std::set<Cost>;

namespace ssp_pdbs {

// Implements a single pattern database
class PatternDatabase {

  // Vector of variables
  Pattern pattern;

  // size of the PDB
  int num_states;

  // number of cost objectives
  size_t num_objectives;

  /*
    final h-values for abstract-states.
    dead-ends are represented by an empty set.
  */
  std::vector<double> distances;

  std::vector<int> generating_op_ids;

  // multipliers for each variable for perfect hash function
  std::vector<int> hash_multipliers;

  std::vector<int> variable_to_index;

  /*
    Computes all abstract operators, builds the match tree (successor
    generator) and then does a Dijkstra regression search to compute
    all final h-values (stored in distances). operator_costs can
    specify individual operator costs for each operator for action
    cost partitioning. If left empty, default operator costs are used.
  */
  void create_pdb(const Problem& problem, const std::vector<Cost>& operator_costs);

  /*
    Project a SAS+ state
   */
  SasPlus::SasPlusState project(SasPlus::SasPlusState state);
  SasPlus::SasPlusPartialState project(SasPlus::SasPlusPartialState state);

  /*
    The given concrete state is used to calculate the index of the
    according abstract state. This is only used for table lookup
    (distances) during search.
  */
  int hash_index(const State& state) const;

  public:
  /*
    Important: It is assumed that the pattern (passed via Options) is
    sorted, contains no duplicates and is small enough so that the
    number of abstract states is below numeric_limits<int>::max()
    Parameters:
     operator_costs: Can specify individual operator costs for each
     operator. This is useful for action cost partitioning. If left
     empty, default operator costs are used.
  */
  PatternDatabase(
    const Problem& problem, const Pattern& pattern,
    const std::vector<Cost>& operator_costs = std::vector<Cost>());
  ~PatternDatabase() = default;

  double get_value(const State& state) const;

  // Returns the pattern (i.e. all variables used) of the PDB
  const Pattern& get_pattern() const {
    return pattern;
  }

  // Returns the size (number of abstract states) of the PDB
  int get_size() const {
    return num_states;
  }
};
} // namespace pdbs

#endif /* PATTERN_DATABASE_H */
