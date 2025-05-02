#include "validation.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>

#include "../representations/sasplus.h"

using namespace std;

namespace ssp_pdbs {
void validate_and_normalize_pattern(const Problem& problem, Pattern& pattern) {
  /*
    - Sort by variable number and remove duplicate variables.
    - Warn if duplicate variables exist.
    - Error if patterns contain out-of-range variable numbers.
  */
  sort(pattern.begin(), pattern.end());
  auto it = unique(pattern.begin(), pattern.end());
  if (it != pattern.end()) {
    pattern.erase(it, pattern.end());
    std::cout << "Warning: duplicate variables in pattern have been removed\n";
  }
  if (!pattern.empty()) {
    if (pattern.front() < 0) {
      std::cout << "Variable number too low in pattern\n";
      exit(1);
    }
    int num_variables = problem.numVariables();
    if (pattern.back() >= num_variables) {
      std::cout << "Variable number too high in pattern\n";
      exit(1);
    }
  }
}

void validate_and_normalize_patterns(const Problem& problem,
                                     PatternCollection& patterns) {
  /*
    - Validate and normalize each pattern (see there).
    - Warn if duplicate patterns exist.
  */
  for (Pattern& pattern : patterns)
    validate_and_normalize_pattern(problem, pattern);
  PatternCollection sorted_patterns(patterns);
  sort(sorted_patterns.begin(), sorted_patterns.end());
  auto it = unique(sorted_patterns.begin(), sorted_patterns.end());
  if (it != sorted_patterns.end()) {
    std::cout << "Warning: duplicate patterns have been detected\n";
  }
}
} // namespace pdbs
