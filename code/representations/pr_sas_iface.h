#ifndef PR_SAS_IFACE
#define PR_SAS_IFACE

#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <cassert>

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/atom_states.h"
#include "../ext/mgpt/rational.h"
#include "../ext/sas-parser/problem.h"
#include "../ext/det_planners/det_pddl_builder.h"
#include "../ssps/constraint_map.h"
#include "../ssps/prob_dist_state.h"


// Doing this (and here instead do down in the file) to avoid YCM/clang to report insane errors in
// pr_sas_iface.cc
#ifdef __clang__
inline bool holdsNegationSafe(state_t const&, atom_t) { assert(false); return false; }
#else
#include "../ext/mgpt/problems.h"
// TODO HACK FIXME: This only works for the main problem being solved, i.e., it
// won't work for any other problem_t generated along the way. Leaving this
// here, instead of the adding it to atom_states.h to make this limitation more
// obvious
inline bool holdsNegationSafe(state_t const& s, atom_t atm) {
  if (gpt::problem->nprec() || atm % 2 == 0) { return s.holds(atm); }
  else { return !s.holds(atm-1); }
}
#endif

// TODO: method that given a MANUALLY translated PDDL file with the all-outcomes
// determinization of the current problem, runs the Fast Downward translator and
// loads up its output

using SasVarValue = int;
static_assert(sizeof(SasVarValue) >= 2 * sizeof(atom_t), "SasVarValue is too small");

using SasVarDomain = std::vector<SasVarValue>;
using FastDwSasProblem = FastDownwardParser::SasProblem;
using MapFdVarPtrToIdx = std::unordered_map<FastDownwardParser::Variable*, size_t>;
using MapFdValToAtomIdx = std::unordered_map<std::string, atom_t>;

class action_t;
using MapFdActionNameToActionTPtr = std::unordered_map<std::string, action_t const*>;

std::string sasVarValueToString(SasVarValue const& val);



class SasVariable {
 public:
  SasVariable() : name_("undefined") { }

  SasVariable(std::string const& name, SasVarDomain domain)
    : name_(name), domain_(domain), has_none_of_those_(false), none_of_those_(0)
  {
    assert(!hasRepeatedValue());
  }

  SasVariable(std::string const& name, SasVarDomain domain, SasVarValue none_of_those)
    // TODO(fwt): efficiency about domain
    : name_(name), domain_(domain), has_none_of_those_(true),
      none_of_those_(none_of_those)
  {
    assert(!hasRepeatedValue());
  }


  std::string const& name() const { return name_; }
  SasVarDomain const& domain() const { return domain_; }

  SasVarValue valueAt(state_t const& s) const {
    assert(checkMutualExclusivityAt(s));
    if (!has_none_of_those_) {
      for (SasVarValue const& v : domain()) {
        if (holdsNegationSafe(s, v)) { return v; }
      }
      std::cout << "State s doesn't have a value for sas variable '" << name()
                << "', that is, mutual exclusivity is broken. Quitting\n";
      exit(-1);
      return domain()[0];
    }
    else {
      for (SasVarValue const& v : domain()) {
        if (v != none_of_those_ && holdsNegationSafe(s, v)) { return v; }
      }
      return none_of_those_;
    }
  }

  bool checkMutualExclusivityAt(state_t const& s) const;

  bool operator==(SasVariable const& other) const {
    return name_ == other.name();
  }

  bool hasNoneOfThoseValue() const { return has_none_of_those_; }
  SasVarValue const& noneOfThoseValue() const { return none_of_those_; }

  bool isValidValue(SasVarValue const& v) const {
    for (SasVarValue const& value : domain()) {
      if (v == value) return true;
    }
    return false;
  }

  int const id() const {
    return id_;
  }

  void set_id(size_t const id) {
    id_ = id;
  }

 private:
  bool hasRepeatedValue() const;
  int id_ = -1;
  std::string name_;
  SasVarDomain domain_;
  bool has_none_of_those_;
  SasVarValue none_of_those_;
};


namespace std {
  template<> struct hash<SasVariable> {
    size_t operator()(SasVariable const& v) const {
      return hash<string>()(v.name());
    }
  };
}

using SasValuation = std::unordered_map<SasVariable, SasVarValue>;

struct VarAndValue {
  SasVariable var;
  SasVarValue value;
};


void changeVarInState(SasVariable const& var, SasVarValue const& v, state_t& s,
                      problem_t const* mgpt_problem = nullptr);


std::ostream& operator<<(std::ostream& os, SasValuation const& valuation);

class SasEffect {
 public:
  SasEffect(VarAndValue const& eff) : eff_(eff) { }
  SasEffect(SasValuation const& cond, VarAndValue const& eff)
    : cond_(cond), eff_(eff)
  { }

  bool isConditional() const { return cond_.size() != 0; }
  bool isApplicable(state_t const& s) const {
    for (auto const& pair : cond())
      if (pair.first.valueAt(s) != pair.second) { return false; }
    return true;
  }

  SasValuation const& cond() const { return cond_; }
  VarAndValue const& eff() const { return eff_; }

 private:
  SasValuation cond_;
  VarAndValue eff_;
};

std::ostream& operator<<(std::ostream& os, SasEffect const& effect);

using VectorSasEffect = std::vector<SasEffect>;


class PrSasAction {
 public:
  PrSasAction() : name_("undefined") { }

//  PrSasAction(std::string const& name, VecRationals cost, SasValuation prec,
//              VecRationals pr, std::vector<SasValuation> eff)
//    : name_(name), cost_(cost), prec_(prec), pr_(pr), eff_(eff)
//  { }

  ~PrSasAction() { }

  std::string const& name() const { return name_; }
  SasValuation const& prec() const { return prec_; }
  VecRationals const& costVector() const { return cost_; }
  size_t size() const { return pr_.size(); }
  Rational const& pr(size_t i) const { return pr_[i]; }
  VectorSasEffect const& eff(size_t i) const { return eff_[i]; }
  action_t const* originalAction() const { return original_action_; }

  bool isApplicable(state_t const& s) const {
    for (auto const& pair : prec()) {
      if (pair.first.valueAt(s) != pair.second) { return false; }
    }
    return true;
  }

  bool operator==(PrSasAction const& other) const {
    // TODO: improve the equality comparison
    return name() == other.name();
  }

  /*
   * Non-const version of the methods above
   */
  std::string& name() { return name_; }
  SasValuation& prec() { return prec_; }
  VecRationals& costVector() { return cost_; }
  void setOriginalAction(action_t const* a) { original_action_ = a; }

  void pushEffect(Rational const& pr, SasValuation const& valuation) {
    pr_.push_back(pr);
    VectorSasEffect eff;
    for (auto const& pair : valuation) {
      eff.push_back(SasEffect({pair.first, pair.second}));
    }
    eff_.push_back(eff);
    assert(pr_.size() == eff_.size());
  }

  void pushEffect(Rational const& pr, VectorSasEffect const& eff) {
    pr_.push_back(pr);
    eff_.push_back(eff);
    assert(pr_.size() == eff_.size());
  }


 private:
  std::string name_;
  VecRationals cost_;
  SasValuation prec_;
  VecRationals pr_;
  std::vector<VectorSasEffect> eff_;
  action_t const* original_action_;
};


// TODO: Terminal cost??
class HackedPrSasProblem {
 public:
  // TODO: ctor receives
  //  - filename for output of the Fast Downward translation
  //  - problem_t to fill up the gaps
  HackedPrSasProblem(problem_t const& problem, bool add_deadend_action);
  ~HackedPrSasProblem() { }

  std::string const& name() const { return name_; }
  std::vector<SasVariable> const& variables() const { return variables_; }
  std::vector<PrSasAction> const& actions() const { return actions_; }
  SasValuation const& initial_state() const { return initial_state_; }
  SasValuation const& goal() const { return goal_; }
  ConstraintMap const& constrs() const { return constrs_; }

  // Equivalent of problem_t::expand. The result is a probability distribution
  // over state_t
  void expand(PrSasAction const& a, state_t const& s, ProbDistStateIface& pr) const;

  bool hasBeenDeadEndTransformed() const { return deadend_trans_applied_; }

  size_t numCostFunctions() const { return 1; }

  SasVariable variable(size_t const id) const { return variables_[id]; }

 private:
  FastDwSasProblem fastDownwardTranslateAndParse(
      DetPDDL const& det_pddl,
      std::string const& translate_bin) const;


  void translateToPrSas(FastDwSasProblem const& det_sas_prob,
                        DetPDDL const& det_pddl);

  /*
   * Build a map from the Fast Downward strings representing an atom value to
   * the id of the mGPT equivalent atom. Here are some examples using the text
   * representation of the mGPT atoms
   * "Atom emptyhand()" -> (emptyhand)
   * "NegatedAtom clear(b4)" -> (not (clear b4))
   * "Atom on(b1, b2)" -> (on b1 b2)
   */
  MapFdValToAtomIdx buildValToAtomTranslationMap() const;

  MapFdVarPtrToIdx translateVariablesAndDomains(
                                          FastDwSasProblem const& det_sas_prob);

  MapFdActionNameToActionTPtr buildNameToActionTranslationMap() const;

  void rebuildActions(FastDwSasProblem const& det_sas_prob,
                      DetPDDL const& det_pddl,
                      MapFdVarPtrToIdx const& fd_varptr_to_var_idx);

  state_t applyVecSasEffTo(VectorSasEffect const& vec_eff, state_t const& s) const;

  void addDeadendAction() {
    PrSasAction deadend_a;
    deadend_a.name() = "deadend-a";
    deadend_a.setOriginalAction(nullptr);
    deadend_a.pushEffect(1, goal_);
    size_t size_cost_vec = actions()[0].costVector().size();
    VecRationals dead_end_penalty(size_cost_vec, Rational(0));
    dead_end_penalty[ACTION_COST] = gpt::dead_end_value;
    for (size_t i = 0; i < constrs_.size(); ++i) {
      dead_end_penalty[constrs_.costIdx(i)] = constrs_.maxExpectedValue(i);
    }
    deadend_a.costVector() = dead_end_penalty;
    actions_.push_back(deadend_a);
    deadend_trans_applied_ = true;
  }

  void dump() const;


  /*
   * Variables
   */
  problem_t const& mgpt_problem_;
  std::string name_;
  bool deadend_trans_applied_;
  std::vector<SasVariable> variables_;
  std::vector<PrSasAction> actions_;
  SasValuation initial_state_;
  SasValuation goal_;
  ConstraintMap constrs_;

  // This variable keeps track of the number of "none of those" SAS+ value for a
  // variable. This is needed to guarantee the HACKish assumption that the
  // pairwise intersection of the variables domain is empty. This is an
  // assumption used in other parts of the, e.g., OperatorCount.
  static int total_none_of_those_;
};



class NonConditionalPrSasAction {
 public:
  NonConditionalPrSasAction() : name_("undefined") { }
  NonConditionalPrSasAction(PrSasAction const& a)
    : name_(a.name()), cost_(a.costVector()), prec_(a.prec()),
      original_sas_action_(&a)
  {
    for (size_t ei = 0; ei < a.size(); ++ei) {
      pr_.push_back(a.pr(ei));
      SasValuation det_eff;
      for (SasEffect const& sas_eff : a.eff(ei)) {
        assert(!sas_eff.isConditional());
        VarAndValue const& change = sas_eff.eff();
        assert(det_eff.find(change.var) == det_eff.end()
                || det_eff[change.var] == change.value);
        det_eff[change.var] = change.value;
      }
      eff_.push_back(det_eff);
    }
  }

  NonConditionalPrSasAction(PrSasAction const& a,
                            SasValuation const& conditioner,
                            std::string const& suffix)
    : name_(a.name() + suffix), cost_(a.costVector()), prec_(a.prec()),
    original_sas_action_(&a)
  {
    for ([[maybe_unused]] auto const& pair : conditioner) {
      assert(prec_.find(pair.first) == prec_.end());
    }
    prec_.insert(conditioner.begin(), conditioner.end());
    for (size_t ei = 0; ei < a.size(); ++ei) {
      pr_.push_back(a.pr(ei));
      SasValuation det_eff;
      for (SasEffect const& sas_eff : a.eff(ei)) {
        bool applicable = true;
        if (sas_eff.isConditional()) {
          for (auto const& pair : sas_eff.cond()) {
            auto it = conditioner.find(pair.first);
            assert (it != conditioner.end());
            if (it->second != pair.second) {
              applicable = false;
              break;
            }
          }
          if (!applicable) continue;
        }
        VarAndValue const& change = sas_eff.eff();
        assert(det_eff.find(change.var) == det_eff.end()
                || det_eff[change.var] == change.value);
        det_eff[change.var] = change.value;
      }
      eff_.push_back(det_eff);
    }
  }

  ~NonConditionalPrSasAction() { }

  std::string const& name() const { return name_; }
  SasValuation const& prec() const { return prec_; }
  VecRationals const& costVector() const { return cost_; }
  size_t size() const { return pr_.size(); }
  Rational const& pr(size_t i) const { return pr_[i]; }
  SasValuation const& eff(size_t i) const { return eff_[i]; }
  action_t const* originalAction() const {
    assert(original_sas_action_);
    return original_sas_action_->originalAction();
  }
  PrSasAction const* originalSasAction() const { return original_sas_action_; }

  bool isApplicable(state_t const& s) const {
    for (auto const& pair : prec()) {
      if (pair.first.valueAt(s) != pair.second) { return false; }
    }
    return true;
  }

  /*
   * Non-const version of the methods above
   */
  // Here pretty much to create the dead-end action
  std::string& name() { return name_; }
  SasValuation& prec() { return prec_; }
  VecRationals& costVector() { return cost_; }
//  void setOriginalAction(action_t const* a) { original_action_ = a; }
  void setOriginalAction(PrSasAction const* a) { original_sas_action_ = a; }

  void pushEffect(Rational const& pr, SasValuation const& eff) {
    pr_.push_back(pr);
    eff_.push_back(eff);
    assert(pr_.size() == eff_.size());
  }

 private:
  std::string name_;
  VecRationals cost_;
  SasValuation prec_;
//  action_t const* original_action_;
  PrSasAction const* original_sas_action_;
  VecRationals pr_;
  std::vector<SasValuation> eff_;
};

class ActionDeterminiser {
 public:
  ActionDeterminiser(HackedPrSasProblem const& sas_problem): sas_problem_(sas_problem)
  {
    for (auto const& action : sas_problem_.actions()) {
      // Remove conditions from each effect
      NonConditionalPrSasAction non_cond_action = NonConditionalPrSasAction(action);
      for (size_t i = 0; i < non_cond_action.size(); ++i) {
        // Construct a deterministic action for each effect
        auto eff = non_cond_action.eff(i);
        auto det_a_i = NonConditionalPrSasAction();
        det_a_i.name() = non_cond_action.name() + std::to_string(i);
        det_a_i.prec() = non_cond_action.prec();
        det_a_i.costVector() = non_cond_action.costVector();
        det_a_i.setOriginalAction(non_cond_action.originalSasAction());
        det_a_i.pushEffect(1.0, eff);
        // and add it to the list of actions
        actions_.push_back(std::move(det_a_i));
      }
    }
  }
  std::vector<NonConditionalPrSasAction> const& actions() const {
    return actions_;
  }
 private:
  HackedPrSasProblem const& sas_problem_;
  std::vector<NonConditionalPrSasAction> actions_;
};

class NonConditionalPrSasProblem {
 public:
  // TODO: ctor receives
  //  - filename for output of the Fast Downward translation
  //  - problem_t to fill up the gaps
  NonConditionalPrSasProblem(HackedPrSasProblem const& sas_problem,
                            bool add_deadend_action);
  ~NonConditionalPrSasProblem() { }

  std::string name() const { return "non-cond-" + sas_problem_.name(); }
  std::vector<NonConditionalPrSasAction> const& actions() const { return actions_; }

  // From HackedPrSasProblem
  std::vector<SasVariable> const& variables() const { return sas_problem_.variables(); }
  SasValuation const& initial_state() const { return sas_problem_.initial_state(); }
  SasValuation const& goal() const { return sas_problem_.goal(); }
  ConstraintMap const& constrs() const { return sas_problem_.constrs(); }

  // Equivalent of problem_t::expand. The result is a probability distribution
  // over state_t
  void expand(NonConditionalPrSasAction const& a, state_t const& s,
              ProbDistStateIface& pr) const;

  bool hasBeenDeadEndTransformed() const { return deadend_trans_applied_; }

  void dump() const;

 private:
  void addDeadendAction() {
    NonConditionalPrSasAction deadend_a;
    deadend_a.name() = "deadend-a";
    deadend_a.setOriginalAction(nullptr);
    deadend_a.pushEffect(1, goal());
    size_t size_cost_vec = actions()[0].costVector().size();
    VecRationals dead_end_penalty(size_cost_vec, Rational(0));
    dead_end_penalty[ACTION_COST] = gpt::dead_end_value;
    for (size_t i = 0; i < constrs().size(); ++i) {
      dead_end_penalty[constrs().costIdx(i)] = constrs().maxExpectedValue(i);
    }
    deadend_a.costVector() = dead_end_penalty;
    actions_.push_back(deadend_a);
    deadend_trans_applied_ = true;
  }

  std::unordered_set<SasVariable> conditionersOfAction(PrSasAction const& a) const {
    std::unordered_set<SasVariable> conditioners;
    for (size_t i = 0; i < a.size(); ++i) {
      VectorSasEffect const& v_eff = a.eff(i);
      for (SasEffect const& eff : v_eff) {
        for (auto const& pair : eff.cond()) {
          conditioners.insert(pair.first);
        }
      }
    }
    return conditioners;
  }

  void compileAwayConditionalEffs(PrSasAction const& a);

  state_t applyValuationTo(SasValuation const& valuation, state_t const& s) const;


  /*
   * Variables
   */
  HackedPrSasProblem const& sas_problem_;
  bool deadend_trans_applied_;
  std::vector<NonConditionalPrSasAction> actions_;
};

#endif  // PR_SAS_IFACE
