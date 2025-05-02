#ifndef PDBS_MATCH_TREE_H_SSP_PDB
#define PDBS_MATCH_TREE_H_SSP_PDB

#include <cstddef>
#include <vector>

#include "../representations/sasplus.h"
#include "types.h"

using FactPair = SasPlus::FactPair;
using Variable = SasPlus::SasPlusVariable;

namespace ssp_pdbs {
/*
  Successor Generator for abstract operators.
*/

class MatchTree {
  // TODO we actually do not need to copy all the variables from the task, but
  // just have a reference for them
  std::vector<Variable> const& variables;
  struct Node;
  // See PatternDatabase for documentation on pattern and hash_multipliers.
  Pattern pattern;
  std::vector<int> hash_multipliers;
  Node* root;
  void insert_recursive(int op_id,
                        const std::vector<FactPair>& regression_preconditions,
                        int pre_index, Node** edge_from_parent);
  void get_applicable_operator_ids_recursive(
    Node* node, int state_index, std::vector<int>& operator_ids) const;
  void dump_recursive(Node* node) const;

  public:
  // Initialize an empty match tree.
  MatchTree(const std::vector<Variable>& variables, const Pattern& pattern,
            const std::vector<int>& hash_multipliers);
  ~MatchTree();
  /* Insert an abstract operator into the match tree, creating or
     enlarging it. */
  void insert(int op_id, const std::vector<FactPair>& regression_preconditions);

  /*
    Extracts all IDs of applicable abstract operators for the abstract state
    given by state_index (the index is converted back to variable/values
    pairs).
  */
  void get_applicable_operator_ids(int state_index,
                                   std::vector<int>& operator_ids) const;
  void dump() const;
};
} // namespace pdbs

#endif
