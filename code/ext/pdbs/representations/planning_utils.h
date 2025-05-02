#pragma once

#include <algorithm>
#include <functional>
#include <numeric>
#include <ranges>
#include <set>
#include <string>
#include <vector>

namespace planning_utils {

using Cost = std::vector<double>;

std::string print(Cost const& cost);

// element-wise addition of two vectors
Cost sum(Cost const& a, Cost const& b);

// MO-admax operator as defined in the ICAPS 22 paper
std::set<Cost> mo_max(std::set<Cost> const& a, std::set<Cost> const& b);

// MO-comax operator as defined in the ICAPS 22 paper
std::set<Cost> co_max(std::set<Cost> const& a, std::set<Cost> const& b);

// MO-plus operator as defined in the ICAPS 22 paper
// TODO fix this definition: it should use a template that takes Cost
template <std::ranges::range T1, std::ranges::range T2>
std::set<Cost> mo_plus(T1&& v1, T2&& v2) {
  std::set<Cost> res;
  for (Cost const& c1 : v1) {
    for (Cost const& c2 : v2) {
      res.insert(sum(c1, c2));
    }
  }
  return res;
}

// returns true if cost is dominated by any cost in cost_set, where the
// domination criterion is given by the domination comparison function
template <typename DomComparison>
bool is_dominated_by(Cost const& cost, std::set<Cost> const& cost_set,
                     DomComparison = DomComparison{});

// filters out vectors dominated by vectors in the domination set
// TODO exchange set to range so that the function does not care about the
// underlying container
template <typename DomComparison>
std::set<Cost> filter_dominated_by(std::set<Cost> const& current,
                                   std::set<Cost> const& domination_set,
                                   DomComparison comp = DomComparison{});

// a is not dominated by b iff for any cost index i, a[i] > b[i] and a != b
struct StrictlyDominated {
  bool operator()(Cost const&, Cost const&);
};

// a is not weakly dominated b b iff for any cost index i, a[i] > b[i]
struct WeaklyDominated {
  bool operator()(Cost const&, Cost const&);
};

} // namespace planning_utils
