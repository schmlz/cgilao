#include "planning_utils.h"

#include <cassert>

using namespace std;

namespace planning_utils {

std::string print(Cost const& cost) {
  std::string res = "(";
  for (double const& c_i : cost) {
    res += std::to_string(c_i) + ",";
  }
  res += ")";
  return res;
}

Cost sum(Cost const& a, Cost const& b) {
  Cost res;
  std::transform(a.begin(), a.end(), b.begin(), std::back_inserter(res),
                 std::plus<>{});
  return res;
}

namespace {
// helper function for mo_max
void mo_max_insert(set<Cost> const& v1, set<Cost> const& v2, set<Cost>& res) {
  // res += v \in ND(vec): \forall v' \in v2: v does not dominate v'
  for (Cost const& v : v1) {
    if (std::none_of(v2.begin(), v2.end(), [&](Cost const& v2_member) {
          return StrictlyDominated()(v, v2_member);
        })) {
      res.insert(v);
    }
  }
}
} // namespace

set<Cost> mo_max(set<Cost> const& v1, set<Cost> const& v2) {
  set<Cost> res;
  // Is this really necessary? Can't we guarantee that the vectors we get are
  // non-dominated?
  auto non_dom_v1 = filter_dominated_by(v1, v1, StrictlyDominated());
  auto non_dom_v2 = filter_dominated_by(v2, v2, StrictlyDominated());
  mo_max_insert(non_dom_v1, non_dom_v2, res);
  mo_max_insert(non_dom_v2, non_dom_v1, res);
  return res;
}

set<Cost> co_max(set<Cost> const& v1, set<Cost> const& v2) {
  set<Cost> res;
  size_t n = (*(v1.begin())).size();
  if (v1.size() == 0) {
    return v2;
  }
  if (v2.size() == 0) {
    return v1;
  }
  for (Cost const& v: v1) {
    for (Cost const& u: v2) {
      Cost w(n);
      for (size_t i=0;i<n;i++) {
        w[i] = std::max(u[i], v[i]);
      }
      res.insert(w);
    }
  }
  res = filter_dominated_by(res, res, StrictlyDominated());
  return res;
}

template <typename DomComp>
bool is_dominated_by(Cost const& cost, set<Cost> const& cost_set,
                     DomComp comp) {
  return std::any_of(cost_set.begin(), cost_set.end(),
                     [&](Cost const& b) { return comp(b, cost); });
}

template <typename DomComp>
set<Cost> filter_dominated_by(set<Cost> const& current,
                              set<Cost> const& domination_set, DomComp comp) {
  // costs a are dominated if any vector in the domination set dominates a
  auto is_dominated = [&](Cost const& cost) {
    return is_dominated_by<DomComp>(cost, domination_set, comp);
  };
  auto result = current;
  std::erase_if(result, is_dominated);
  return result;
}

bool StrictlyDominated::operator()(Cost const& a, Cost const& b) {
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i] > b[i]) {
      return false;
    }
  }
  return a != b;
}

bool WeaklyDominated::operator()(Cost const& a, Cost const& b) {
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i] > b[i]) {
      return false;
    }
  }
  return true;
}

template bool is_dominated_by<StrictlyDominated>(Cost const& cost,
                                                 set<Cost> const& cost_set,
                                                 StrictlyDominated comp);

template bool is_dominated_by<WeaklyDominated>(Cost const& cost,
                                               set<Cost> const& cost_set,
                                               WeaklyDominated comp);

template set<Cost> filter_dominated_by<StrictlyDominated>(set<Cost> const&,
                                                          set<Cost> const&,
                                                          StrictlyDominated);

template set<Cost> filter_dominated_by<WeaklyDominated>(set<Cost> const&,
                                                        set<Cost> const&,
                                                        WeaklyDominated);

} // namespace planning_utils
