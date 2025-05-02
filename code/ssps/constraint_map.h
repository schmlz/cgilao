#ifndef CONSTRAINT_MAP_F
#define CONSTRAINT_MAP_F

#include <iostream>
#include <vector>

class ConstraintMap {
 public:
  ConstraintMap() { }

  void insert(std::string const& name, size_t cost_idx, double max_expected_val,
              double deadend_penalty)
  {
    constr_.push_back({name, cost_idx, max_expected_val, deadend_penalty});
  }
  size_t size() const { return constr_.size(); }
  size_t costIdx(size_t i) const { return constr_[i].cost_idx; }
  double maxExpectedValue(size_t i) const { return constr_[i].max_expected_val;}
  double deadendPenalty(size_t i) const { return constr_[i].deadend_penalty;}
  std::string const& name(size_t i) const { return constr_[i].name; }

 private:
  struct ConstrainData {
    std::string name;
    size_t cost_idx;
    double max_expected_val;
    double deadend_penalty;
  };

  std::vector<ConstrainData> constr_;
};


#endif
