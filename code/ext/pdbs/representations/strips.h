#ifndef SRC_REPRESENTATIONS_STRIPS_PROBLEMS_H__PDB
#define SRC_REPRESENTATIONS_STRIPS_PROBLEMS_H__PDB

#include <cassert>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <boost/dynamic_bitset.hpp>

/**
 * Namespace for all the models and data-structures to represent STRIPS models.
 *
 * The focus of this implementation is a trade-off of simplicity and efficiency instead
 * of absolute efficiency.
 */
namespace STRIPS {
  using StripsState = boost::dynamic_bitset<>;
  using Mask = boost::dynamic_bitset<>;

  /**
   * Struct representing a STRIPS effect
   */
  struct Effect {
   public:
    // Create an empty effect for a problem with n propositions
    explicit Effect(size_t n) : del(n), add(n) { }

    Effect(Mask const& d, Mask const& a) : del(d), add(a) { }

    StripsState apply(StripsState const& s) const {
      return (s & ~del) | add;
    }

    bool operator==(Effect const& other) const {
      return del == other.del && add == other.add;
    }

    Mask del;
    Mask add;
  };

  using PrState = std::map<StripsState, double>;
  using PrEffect = std::vector<Effect>;
  using PrDist = std::vector<double>;

  /**
   * Class representing a probabilistic STRIPS action
   */
  class StripsAction {
   public:

    using PrState = std::map<StripsState, double>;
    using PrEffect = std::vector<Effect>;
    using PrDist = std::vector<double>;

    using Cost = std::vector<double>;  // a cost vector
    using Value = std::set<Cost>;  // a collection of costs associated to a state
    using ValueMap = std::map<StripsState, Value>;  // value map for a problem

    friend class StripsMOSSP;

    StripsAction(std::string const& name,
                 Cost cost,
                 Mask prec,
                 PrEffect const& pr_effects,
                 PrDist const& pr_probs)
      : name_(name), cost_(cost), prec_(prec), pr_effects_(pr_effects), pr_probs_(pr_probs)
    {
      assert(pr_effects.size() == pr_probs.size());
    }

    bool isApplicable(StripsState const& s) const {
      return (prec_ & s) == prec_;
    }

    size_t numCostFunctions() const {
      return cost_.size();
    }

    Cost cost() const {
      return cost_;
    }

    Cost cost(StripsState const& s) const {
      return cost_;
    }

    PrState successors_prob(StripsState const& s) const {
//      assert(isApplicable(s));
      PrState succ;  // map<state, double>
      for (size_t i = 0; i < pr_effects_.size(); i++) {
        auto const& p = pr_effects_[i];
        succ[p.apply(s)] += pr_probs_[i];
      }
      return succ;
    }

    std::set<StripsState> successors(StripsState const& s) const {
//      assert(isApplicable(s));
      std::set<StripsState> succ;
      for (const auto & p : pr_effects_) {
        succ.insert(p.apply(s));
      }
      return succ;
    }

    std::string const& name() const { return name_; }

    Mask precondition() const { return prec_; }

    PrEffect pr_effects() const { return pr_effects_; }

    bool operator< (const StripsAction& other) const { return name_ < other.name(); }

    bool operator> (const StripsAction& other) const { return name_ > other.name(); }

   private:
    std::string name_;
    Cost cost_;
    Mask prec_;
    PrEffect pr_effects_;
    PrDist pr_probs_;
  };


  /**
   * Class representing a STRIPS-based MOSSP
   */
  class StripsMOSSP {
   public:
    using State = StripsState;
    using Action = StripsAction;

    using PrState = std::map<StripsState, double>;
    using PrEffect = std::vector<Effect>;
    using PrDist = std::vector<double>;

    using Cost = std::vector<double>;  // a cost vector
    using Value = std::set<Cost>;  // a collection of costs associated to a state
    using ValueMap = std::map<StripsState, Value>;  // value map for a problem

    StripsMOSSP(std::string name,
                State s0,
                Mask goal_set,
                std::vector<Action> const& actions,
                std::map<size_t, std::string> idx_to_name)
      : name_(name), s0_(s0), goal_set_(goal_set), actions_(actions)
    {
      assert(!actions_.empty());
      [[maybe_unused]] size_t n_prop = numPropositions();
      [[maybe_unused]] size_t n_cost_func = numCostFunctions();
      assert(s0_.size() == n_prop);
      assert(goal_set_.size() == n_prop);
      max_effects_ = 1;
      for (Action const& a : actions_) {
        assert(a.precondition().size() == n_prop);
        assert(a.numCostFunctions() == n_cost_func);
        max_effects_ = std::max(max_effects_, a.pr_effects_.size());
#ifndef NDEBUG
        for (auto const& p : a.pr_effects_) {
          assert(p.del.size() == n_prop);
          assert(p.add.size() == n_prop);
        }
#endif
      }
      // remove parenthesis from propositions for reading strips problems
      for (const auto& kv: idx_to_name) {
        std::string predicate = kv.second;
        predicate = predicate.substr(1, predicate.size() - 2);  // remove parenthesis
        idx_to_name_[kv.first] = predicate;
      }
      for (const auto& kv: idx_to_name) {
        std::string predicate = kv.second;
        predicate = predicate.substr(1, predicate.size() - 2);
        name_to_idx_[predicate] = kv.first;
      }
    }

    StripsMOSSP() = default;

    State initialState() const {
      return s0_;
    }

    bool isGoal(State const& s) const {
      return (s | ~goal_set_).all();
    }

    size_t numActions() const {
      return actions_.size();
    }

    std::vector<Action> const& allActions() const {
      return actions_;
    }

    std::vector<Action> applicableActions(State const& s) const {
      std::vector<Action> applicable;
      for (Action const& a : actions_) {
        if (a.isApplicable(s)) { applicable.push_back(a); }
      }
      return applicable;
    }

    size_t numCostFunctions() const {
      auto it = actions_.begin();
      if (it != actions_.end()) {
        return it->numCostFunctions();
      }
      return 0;
    }

    Cost cost(State const& s, Action const& a) const {
      return a.cost(s);
    }

    Cost cost(Action const& a) const {
      return a.cost();
    }

    PrState successors_prob(State const& s, Action const& a) const {
      return a.successors_prob(s);
    }

    std::set<State> successors(State const& s, Action const& a) const {
      return a.successors(s);
    }

    Mask const& goalSet() const {
      return goal_set_;
    }

    size_t numPropositions() const {
      return s0_.size();
    }

    size_t numMaxEffects() const {
      return max_effects_;
    }

    std::string to_string(State const& s) const {
      std::ostringstream ost;
      bool first = true;
      for (size_t idx = s.find_first(); idx != s.npos; idx = s.find_next(idx)) {
        assert(idx_to_name_.find(idx) != idx_to_name_.end());
        ost << (first ? "" : " ") << idx_to_name_.find(idx)->second;
        first = false;
      }
      return ost.str();
    }

    std::string to_string(Action const& a) const {
      return a.name();
    }

    std::string get_name(Action const& a) const {
      return a.name();
    }

    std::map<std::string, size_t> get_name_to_idx() const {
      return name_to_idx_;
    }

    std::map<size_t, std::string> get_idx_to_name() const {
      return idx_to_name_;
    }

    std::string name() const {
      return name_;
    }

    void print(State const& s) {
      std::cout<<s<<std::endl;
    }

   private:
    std::string name_;
    State s0_;
    Mask goal_set_;
    std::vector<Action> actions_;
    std::map<size_t, std::string> idx_to_name_;
    std::map<std::string, size_t> name_to_idx_;
    size_t max_effects_;
  };


  /**
   * Class representing a policy: maps states to sets of (convex) Pareto optimal actions
   */

}  // namespace STRIPS

#endif  // SRC_REPRESENTATIONS_STRIPS_PROBLEMS_H_
