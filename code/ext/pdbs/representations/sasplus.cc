#include "sasplus.h"


/*** SasPlusActionDet ***/
SasPlus::SasPlusActionDet::SasPlusActionDet(const std::string &name, const SasPlus::SasPlusActionDet::Cost &cost,
                                            const SasPlus::SasPlusPartialState &prec,
                                            const SasPlus::SasPlusActionDet::Effect &effect, int index)
        : index(index), name_(name), cost_(cost), prec_(prec), effect_(effect) { }

SasPlus::SasPlusActionDet::SasPlusActionDet(const std::string &name, const SasPlus::SasPlusActionDet::Cost &cost,
                                            const SasPlus::SasPlusPartialState &prec,
                                            const SasPlus::SasPlusActionDet::Effect &effect)
        : index(0), name_(name), cost_(cost), prec_(prec), effect_(effect) { }

bool SasPlus::SasPlusActionDet::isApplicable(SasPlusState const& s) const {
  return contains(prec_, s);
}

size_t SasPlus::SasPlusActionDet::numCostFunctions() const {
  return cost_.size();
}

const SasPlus::SasPlusActionDet::Cost &SasPlus::SasPlusActionDet::cost() const {
  return cost_;
}

SasPlus::SasPlusActionDet::Cost SasPlus::SasPlusActionDet::cost(const SasPlus::SasPlusState &s) const {
  return cost_;
}

SasPlus::SasPlusState SasPlus::SasPlusActionDet::successor(const SasPlus::SasPlusState &s) const {
//      assert(isApplicable(s));
  SasPlusState sp(s);
  for (auto const& it : effect_) {
    sp[it.first] = it.second;
  }
  return sp;
}

SasPlus::SasPlusActionDet::PrState SasPlus::SasPlusActionDet::successors_prob(const SasPlus::SasPlusState &s) const {
//      assert(isApplicable(s));
  return {{successor(s), 1}};
}

std::set<SasPlus::SasPlusState> SasPlus::SasPlusActionDet::successors(const SasPlus::SasPlusState &s) const {
//      assert(isApplicable(s));
  return {successor(s)};
}

int SasPlus::SasPlusActionDet::get_id() const { return index; }

std::string const &SasPlus::SasPlusActionDet::name() const { return name_; }

const SasPlus::SasPlusPartialState &SasPlus::SasPlusActionDet::precondition() const { return prec_; }

const SasPlus::SasPlusActionDet::Effect &SasPlus::SasPlusActionDet::effect() const { return effect_; }

bool SasPlus::SasPlusActionDet::operator<(const SasPlus::SasPlusActionDet &other) const { return name_ < other.name(); }

bool SasPlus::SasPlusActionDet::operator>(const SasPlus::SasPlusActionDet &other) const { return name_ > other.name(); }


/*** SasPlusAction ***/
SasPlus::SasPlusAction::SasPlusAction(const std::string &name, const SasPlus::SasPlusAction::Cost &cost,
                                      const SasPlus::SasPlusPartialState &prec,
                                      const SasPlus::SasPlusAction::PrEffect &pr_effects,
                                      const SasPlus::SasPlusAction::PrDist &pr_probs, int index)
        : index(index), name_(name), cost_(cost), prec_(prec), pr_effects_(pr_effects), pr_probs_(pr_probs)
{
  assert(pr_effects.size() == pr_probs.size());
}

SasPlus::SasPlusAction::SasPlusAction(const std::string &name, const SasPlus::SasPlusAction::Cost &cost,
                                      const SasPlus::SasPlusPartialState &prec,
                                      const SasPlus::SasPlusAction::PrEffect &pr_effects,
                                      const SasPlus::SasPlusAction::PrDist &pr_probs)
        : index(0), name_(name), cost_(cost), prec_(prec), pr_effects_(pr_effects), pr_probs_(pr_probs)
{
  assert(pr_effects.size() == pr_probs.size());
}


bool SasPlus::SasPlusAction::isApplicable(SasPlusState const& s) const {
  return contains(prec_, s);
}

size_t SasPlus::SasPlusAction::numCostFunctions() const {
  return cost_.size();
}

const SasPlus::SasPlusAction::Cost &SasPlus::SasPlusAction::cost() const {
  return cost_;
}

SasPlus::SasPlusAction::Cost SasPlus::SasPlusAction::cost(const SasPlus::SasPlusState &s) const {
  return cost_;
}

SasPlus::SasPlusAction::PrState SasPlus::SasPlusAction::successors_prob(const SasPlus::SasPlusState &s) const {
//      assert(isApplicable(s));
  PrState succ;
  for (size_t i = 0; i < pr_effects_.size(); i++) {
    auto const& eff = pr_effects_[i];
    SasPlusState sp(s);
    for (auto const& it : eff) {
      sp[it.first] = it.second;
    }
    succ[sp] += pr_probs_[i];
  }
  return succ;
}

std::set<SasPlus::SasPlusState> SasPlus::SasPlusAction::successors(const SasPlus::SasPlusState &s) const {
//      assert(isApplicable(s));
  std::set<SasPlusState> succ;
  for (const auto & eff : pr_effects_) {
    SasPlusState sp(s);
    for (auto const& it : eff) {
      sp[it.first] = it.second;
    }
    succ.insert(sp);
  }
  return succ;
}

int SasPlus::SasPlusAction::get_id() const { return index; }

std::string const &SasPlus::SasPlusAction::name() const { return name_; }

const SasPlus::SasPlusPartialState &SasPlus::SasPlusAction::precondition() const { return prec_; }

const SasPlus::SasPlusAction::PrEffect &SasPlus::SasPlusAction::pr_effects() const { return pr_effects_; }

const SasPlus::SasPlusAction::PrDist &SasPlus::SasPlusAction::pr_dist() const { return pr_probs_; }

std::vector<SasPlus::SasPlusActionDet> SasPlus::SasPlusAction::determinise() {
  std::vector<SasPlus::SasPlusActionDet> ret;
  for (size_t i = 0; i < pr_effects_.size(); i++) {
    ret.push_back({name_+"-det-"+std::to_string(i), cost_, prec_, pr_effects_[i]});
  }
  return ret;
}

bool SasPlus::SasPlusAction::operator<(const SasPlus::SasPlusAction &other) const {
  return std::tie(prec_, cost_, pr_effects_, pr_probs_) < std::tie(other.prec_, other.cost_, other.pr_effects_, other.pr_probs_);
}

bool SasPlus::SasPlusAction::operator>(const SasPlus::SasPlusAction &other) const {
  return std::tie(prec_, cost_, pr_effects_, pr_probs_) >
         std::tie(other.prec_, other.cost_, other.pr_effects_, other.pr_probs_);
}

bool SasPlus::SasPlusAction::operator==(const SasPlus::SasPlusAction &other) const {
  return prec_ == other.prec_ && cost_ == other.cost_ && pr_effects_ == other.pr_effects_ && pr_probs_ == other.pr_probs_;
}


/*** SasPlusMOSSPDet ***/
SasPlus::SasPlusMOSSPDet::SasPlusMOSSPDet(std::string name, const std::vector<SasPlusVariable> &variables,
                                          const SasPlus::SasPlusMOSSPDet::State &s0, const SasPlus::SasPlusPartialState &goal,
                                          const std::vector<Action> &actions)
        : name_(name), s0_(s0), goal_(goal), actions_(actions), variables_(variables)
{
  max_effects_ = 1;
#ifndef NDEBUG
  assert(!actions_.empty());
  size_t n_var = numVariables();
  size_t n_cost_func = numCostFunctions();
  assert(s0_.size() == n_var);
  for (Action const& a : actions_) {
    assert(a.numCostFunctions() == n_cost_func);
  }
#endif
}

SasPlus::SasPlusMOSSPDet::State SasPlus::SasPlusMOSSPDet::initialState() const {
  return s0_;
}

bool SasPlus::SasPlusMOSSPDet::isGoal(const SasPlus::SasPlusMOSSPDet::State &s) const {
  return contains(goal_, s);
}

size_t SasPlus::SasPlusMOSSPDet::numActions() const {
  return actions_.size();
}

const std::vector<SasPlus::SasPlusActionDet> &SasPlus::SasPlusMOSSPDet::allActions() const {
  return actions_;
}

std::vector<SasPlus::SasPlusActionDet> SasPlus::SasPlusMOSSPDet::applicableActions(const SasPlus::SasPlusMOSSPDet::State &s) const {
  std::vector<Action> result;
  for (Action const& action : actions_) {
    if (action.isApplicable(s)) { result.push_back(action); }
  }
  return result;
}

size_t SasPlus::SasPlusMOSSPDet::numCostFunctions() const {
  auto it = actions_.begin();
  if (it != actions_.end()) {
    return it->numCostFunctions();
  }
  return 0;
}

SasPlus::SasPlusMOSSPDet::Cost SasPlus::SasPlusMOSSPDet::cost(const SasPlus::SasPlusMOSSPDet::State &s, const SasPlus::SasPlusMOSSPDet::Action &a) const {
  return a.cost(s);
}

SasPlus::SasPlusMOSSPDet::Cost SasPlus::SasPlusMOSSPDet::cost(const SasPlus::SasPlusMOSSPDet::Action &a) const {
  return a.cost();
}

SasPlus::SasPlusState SasPlus::SasPlusMOSSPDet::successor(const SasPlus::SasPlusMOSSPDet::State &s, const SasPlus::SasPlusMOSSPDet::Action &a) const {
  return a.successor(s);
}

SasPlus::SasPlusMOSSP::PrState SasPlus::SasPlusMOSSPDet::successors_prob(const SasPlus::SasPlusMOSSPDet::State &s,
                                                                      const SasPlus::SasPlusMOSSPDet::Action &a) const {
  return a.successors_prob(s);
}

std::set<SasPlus::SasPlusState> SasPlus::SasPlusMOSSPDet::successors(const SasPlus::SasPlusMOSSPDet::State &s, const SasPlus::SasPlusMOSSPDet::Action &a) const {
  return a.successors(s);
}

size_t SasPlus::SasPlusMOSSPDet::numVariables() const {
  return variables_.size();
}

size_t SasPlus::SasPlusMOSSPDet::numMaxEffects() const {
  return max_effects_;
}

SasPlus::SasPlusVariable SasPlus::SasPlusMOSSPDet::variable(size_t i) const {
  assert(i < variables_.size());
  return variables_[i];
}

const std::vector<SasPlus::SasPlusVariable> &SasPlus::SasPlusMOSSPDet::variables() const {
  return variables_;
}

const SasPlus::SasPlusPartialState &SasPlus::SasPlusMOSSPDet::goal() const {
  return goal_;
}

std::string SasPlus::SasPlusMOSSPDet::to_string(const SasPlus::SasPlusMOSSPDet::State &s) const {
  std::string res = "";
  for (size_t i = 0; i < s.size(); ++i) {
    SasPlusVariable v = variable(i);
    res += "(" + v.name + "," + std::to_string(s[i]) + ") ";
  }
  return res;
}

std::string SasPlus::SasPlusMOSSPDet::to_string(const SasPlus::SasPlusMOSSPDet::Action &a) const {
  return a.name();
}

std::string SasPlus::SasPlusMOSSPDet::get_name(const SasPlus::SasPlusMOSSPDet::Action &a) const {
  return a.name();
}

std::string const &SasPlus::SasPlusMOSSPDet::name() const { return name_; }


/*** SasPlusMOSSP ***/
SasPlus::SasPlusMOSSP::SasPlusMOSSP(std::string name, const std::vector<SasPlusVariable> &variables,
                                    const SasPlus::SasPlusMOSSP::State &s0, const SasPlus::SasPlusPartialState &goal,
                                    const std::vector<Action> &actions)
        : name_(name), s0_(s0), goal_(goal), actions_(actions), variables_(variables)
{
  max_effects_ = 1;
#ifndef NDEBUG
  assert(!actions_.empty());
  size_t n_var = numVariables();
  size_t n_cost_func = numCostFunctions();
  assert(s0_.size() == n_var);
  for (Action const& a : actions_) {
    assert(a.numCostFunctions() == n_cost_func);
    max_effects_ = std::max(max_effects_, a.pr_effects_.size());
  }
#endif
}

SasPlus::SasPlusMOSSP::State SasPlus::SasPlusMOSSP::initialState() const {
  return s0_;
}

bool SasPlus::SasPlusMOSSP::isGoal(const SasPlus::SasPlusMOSSP::State &s) const {
  return contains(goal_, s);
}

size_t SasPlus::SasPlusMOSSP::numActions() const {
  return actions_.size();
}

const std::vector<SasPlus::SasPlusAction> &SasPlus::SasPlusMOSSP::allActions() const {
  return actions_;
}

std::vector<SasPlus::SasPlusAction> SasPlus::SasPlusMOSSP::applicableActions(const SasPlus::SasPlusMOSSP::State &s) const {
  std::vector<Action> result;
  for (Action const& action : actions_) {
    if (action.isApplicable(s)) { result.push_back(action); }
  }
  return result;
}

size_t SasPlus::SasPlusMOSSP::numCostFunctions() const {
  auto it = actions_.begin();
  if (it != actions_.end()) {
    return it->numCostFunctions();
  }
  return 0;
}

SasPlus::SasPlusMOSSP::Cost SasPlus::SasPlusMOSSP::cost(const SasPlus::SasPlusMOSSP::State &s, const SasPlus::SasPlusMOSSP::Action &a) const {
  return a.cost(s);
}

SasPlus::SasPlusMOSSP::Cost SasPlus::SasPlusMOSSP::cost(const SasPlus::SasPlusMOSSP::Action &a) const {
  return a.cost();
}

SasPlus::SasPlusMOSSP::PrState SasPlus::SasPlusMOSSP::successors_prob(const SasPlus::SasPlusMOSSP::State &s,
                                                                      const SasPlus::SasPlusMOSSP::Action &a) const {
  return a.successors_prob(s);
}

std::set<SasPlus::SasPlusState> SasPlus::SasPlusMOSSP::successors(const SasPlus::SasPlusMOSSP::State &s, const SasPlus::SasPlusMOSSP::Action &a) const {
  return a.successors(s);
}

size_t SasPlus::SasPlusMOSSP::numVariables() const {
  return variables_.size();
}

size_t SasPlus::SasPlusMOSSP::numMaxEffects() const {
  return max_effects_;
}

SasPlus::SasPlusVariable SasPlus::SasPlusMOSSP::variable(size_t i) const {
  assert(i < variables_.size());
  return variables_[i];
}

const std::vector<SasPlus::SasPlusVariable> &SasPlus::SasPlusMOSSP::variables() const {
  return variables_;
}

const SasPlus::SasPlusPartialState &SasPlus::SasPlusMOSSP::goal() const {
  return goal_;
}

std::string SasPlus::SasPlusMOSSP::to_string(const SasPlus::SasPlusPartialState &s) const {
  std::string res = "";
  for (auto const& kv: s) {
    SasPlusVariable v = variable(kv.first);
    res += "(" + v.name + "," + std::to_string(kv.second) + ") ";
  }
  return res;
}

std::string SasPlus::SasPlusMOSSP::to_string(const SasPlus::SasPlusMOSSP::State &s) const {
  std::string res = "";
  for (size_t i = 0; i < s.size(); ++i) {
    SasPlusVariable v = variable(i);
    res += "(" + v.name + "," + std::to_string(s[i]) + ") ";
  }
  return res;
}

std::string SasPlus::SasPlusMOSSP::to_string(const SasPlus::SasPlusMOSSP::Action &a) const {
  return a.name();
}

void SasPlus::SasPlusMOSSP::print(const SasPlus::SasPlusPartialState &s) const {
  std::cout<<to_string(s)<<std::endl;
}

void SasPlus::SasPlusMOSSP::print(const SasPlus::SasPlusMOSSP::State &s) const {
  std::cout<<to_string(s)<<std::endl;
}

void SasPlus::SasPlusMOSSP::print(const SasPlus::SasPlusMOSSP::Action &a) const {
  std::cout<<a.name()<<std::endl;
  std::cout<<"prec:\n";
  print(a.precondition());
  for (size_t i = 0; i < a.pr_effects_.size(); i++) {
    std::cout<<"eff"<<i<<": "<<a.pr_probs_[i]<<"\n";
    print(a.pr_effects_[i]);
  }
}

std::string SasPlus::SasPlusMOSSP::get_name(const SasPlus::SasPlusMOSSP::Action &a) const {
  return a.name();
}

std::string const &SasPlus::SasPlusMOSSP::name() const { return name_; }

SasPlus::SasPlusMOSSPDet SasPlus::SasPlusMOSSP::determinise() {
  std::vector<SasPlus::SasPlusActionDet> det_actions;
  for (auto action: actions_) {
    for (auto det_action: action.determinise()) {
      det_actions.push_back(det_action);
    }
  }
  return {name_+"-det", variables_, s0_, goal_, det_actions};
}