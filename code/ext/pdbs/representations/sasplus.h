#ifndef SRC_REPRESENTATIONS_SASPLUS_H__PDB
#define SRC_REPRESENTATIONS_SASPLUS_H__PDB

#include <cassert>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <set>

#include <cstdint>

/**
 * Namespace for all the models and data-structures to represent SAS+ models.
 *
 * The focus of this implementation is a trade-off of simplicity and efficiency
 * instead of absolute efficiency
 */
namespace SasPlus {

  /// Type to represent the size of the domain of a variable as well as the
  /// maximum value of a variable (max_size - 1). Notice that uint8_t should also
  /// work but the memory savings is minimal due to the / overhead of std::vector
  /// and std::map/unordered_map
  using VariableDomSize = uint16_t;

  /// A SAS+ variables is represented by its size of s domain (its possible values
  /// are {0, ..., domain-size - 1}) and their name.
  struct SasPlusVariable {
    int index;
    std::string name;
    VariableDomSize domain;

    VariableDomSize get_domain_size() const {
        return domain;
    }

    bool operator <(const SasPlusVariable& other) const {
      return index < other.index;
    }
  };

  /// The i-th position contains the value of the SAS+ variable v_i
  using SasPlusState = std::vector<VariableDomSize>;

  using FactPair = std::pair<int, VariableDomSize>;

  /// Map from i to a domain value representing the value of the SAS+ variable
  /// v_i. Map is used instead of unordered_map because single-entry look-up usage
  /// is minimal, instead the common usage is to iterate over all values
  /// Seems to be used only for goal condition, preconditions and effects.
  using SasPlusPartialState = std::map<size_t, VariableDomSize>;

  /// Test if the SAS+ state s contains the partial SAS+ state
  inline bool contains(SasPlusPartialState const& partial_state, SasPlusState s) {
    for (auto const& it : partial_state) {
      size_t const& idx = it.first;
      VariableDomSize const& value = it.second;
      assert(idx < s.size());
      if (s[idx] != value) {
        return false;
      }
    }
    return true;
  }

//  void print(SasPlusPartialState s) {
//    std::cout<<"printing s:\n";
//    for (auto const &kv: s) {
//      std::cout<<fmt::format("({},{}) ", kv.first, kv.second)<<std::endl;
//    }
//  }
//
//  void print(SasPlusState s) {
//    std::cout<<"printing s:\n";
//    for (size_t i = 0; i < s.size(); ++i) {
//      std::cout<<fmt::format("({},{}) ", i, s[i])<<std::endl;
//    }
//  }

  /**
   * Class to represent a ///deterministic/// SAS+ action with arbitrary cost. Notice that the prevail
   * condition is represented together with the precondition in this class.
   */
  class SasPlusActionDet {
   public:

    using PrState = std::map<SasPlusState, double>;
    using Effect = SasPlusPartialState;

    using Cost = std::vector<double>;  // a cost vector
    using Value = std::set<Cost>;  // a collection of costs associated to a state
    using ValueMap = std::map<SasPlusState, Value>;  // value map for a problem

    friend class SasPlusMOSSP;

    SasPlusActionDet(std::string const& name,
                     Cost const& cost,
                     SasPlusPartialState const& prec,
                     Effect const& effect,
                     int index);

    SasPlusActionDet(std::string const& name,
                     Cost const& cost,
                     SasPlusPartialState const& prec,
                     Effect const& effect);

    bool isApplicable(SasPlusState const& s) const;

    size_t numCostFunctions() const;

    Cost const& cost() const;

    Cost cost(SasPlusState const& s) const;

    SasPlus::SasPlusState successor(SasPlusState const& s) const;

    PrState successors_prob(SasPlusState const& s) const;  // so solver can run

    std::set<SasPlusState> successors(SasPlusState const& s) const;  // so solver can run

    int get_id() const;

    std::string const& name() const;

    SasPlusPartialState const& precondition() const;

    Effect const& effect() const;

    bool operator< (const SasPlusActionDet& other) const;

    bool operator> (const SasPlusActionDet& other) const;

   private:
    int index;
    std::string name_;
    Cost cost_;
    SasPlusPartialState prec_;
    Effect effect_;
  };

  /**
   * Class to represent a SAS+ action with arbitrary cost. Notice that the prevail
   * condition is represented together with the precondition in this class.
   */
  class SasPlusAction {
    public:

    using PrState = std::map<SasPlusState, double>;
    using PrEffect = std::vector<SasPlusPartialState>;
    using PrDist = std::vector<double>;

    using Cost = std::vector<double>;  // a cost vector
    using Value = std::set<Cost>;  // a collection of costs associated to a state
    using ValueMap = std::map<SasPlusState, Value>;  // value map for a problem

    friend class SasPlusMOSSP;

    SasPlusAction(std::string const& name,
                  Cost const& cost,
                  SasPlusPartialState const& prec,
                  PrEffect const& pr_effects,
                  PrDist const& pr_probs,
                  int index);

    SasPlusAction(std::string const& name,
                  Cost const& cost,
                  SasPlusPartialState const& prec,
                  PrEffect const& pr_effects,
                  PrDist const& pr_probs);

    bool isApplicable(SasPlusState const& s) const;

    size_t numCostFunctions() const;

    Cost const& cost() const;

    Cost cost(SasPlusState const& s) const;

    PrState successors_prob(SasPlusState const& s) const;

    std::set<SasPlusState> successors(SasPlusState const& s) const;

    int get_id() const;

    std::string const& name() const;

    SasPlusPartialState const& precondition() const;

    PrEffect const& pr_effects() const;
    PrDist const& probabilities() const { return pr_probs_; }

    PrDist const& pr_dist() const;

    std::vector<SasPlusActionDet> determinise();

    bool operator< (const SasPlusAction& other) const;

    bool operator> (const SasPlusAction& other) const;

    bool operator== (const SasPlusAction& other) const;

    private:
    int index;
    std::string name_;
    Cost cost_;
    SasPlusPartialState prec_;
    PrEffect pr_effects_;
    PrDist pr_probs_;
  };

  /**
   * Class to represent a deterministic SAS+ problem with vector costs.
   */
  class SasPlusMOSSPDet {
   public:
    using PrState = std::map<SasPlusState, double>;
    using State = SasPlusState;
    using Action = SasPlusActionDet;

    using Cost = std::vector<double>;  // a cost vector
    using Value = std::set<Cost>;  // a collection of costs associated to a state
    using ValueMap = std::map<SasPlusState, Value>;  // value map for a problem

    SasPlusMOSSPDet(std::string name,
                    std::vector<SasPlusVariable> const& variables,
                    State const& s0,
                    SasPlusPartialState const& goal,
                    std::vector<Action> const& actions);

    SasPlusMOSSPDet() = default;

    State initialState() const;

    bool isGoal(State const& s) const;

    size_t numActions() const;

    std::vector<Action> const& allActions() const;

    std::vector<Action> applicableActions(State const& s) const;

    size_t numCostFunctions() const;

    Cost cost(State const& s, Action const& a) const;

    Cost cost(Action const& a) const;

    PrState successors_prob(State const& s, Action const& a) const;  // so solver can run

    std::set<State> successors(State const& s, Action const& a) const;  // so solver can run

    State successor(State const& s, Action const& a) const;

    /*
     * SAS+ specific methods to handle variables and the goal formula
     */
    size_t numVariables() const;

    size_t numMaxEffects() const;

    SasPlusVariable variable(size_t i) const;

    /// Use this method in a range-based for loop to iterate over all variables
    std::vector<SasPlusVariable> const& variables() const;

    SasPlusPartialState const& goal() const;

    std::string to_string(State const& s) const;

    std::string to_string(Action const& a) const;

    std::string get_name(Action const& a) const;

    std::string const& name() const;

   private:
    std::string name_;
    State s0_;
    SasPlusPartialState goal_;
    std::vector<Action> actions_;
    std::vector<SasPlusVariable> variables_;
    size_t max_effects_;
  };


  /**
   * Class to represent a stochastic SAS+ problem with vector costs.
   */
  class SasPlusMOSSP {
   public:
    using State = SasPlusState;
    using Action = SasPlusAction;

    using PrState = std::map<SasPlusState, double>;
    using PrEffect = std::vector<SasPlusPartialState>;
    using PrDist = std::vector<double>;

    using Cost = std::vector<double>;  // a cost vector
    using Value = std::set<Cost>;  // a collection of costs associated to a state
    using ValueMap = std::map<SasPlusState, Value>;  // value map for a problem

    SasPlusMOSSP(std::string name,
                 std::vector<SasPlusVariable> const& variables,
                 State const& s0,
                 SasPlusPartialState const& goal,
                 std::vector<Action> const& actions);

    SasPlusMOSSP() = default;

    State initialState() const;

    bool isGoal(State const& s) const;

    size_t numActions() const;

    std::vector<Action> const& allActions() const;

    std::vector<Action> applicableActions(State const& s) const;

    size_t numCostFunctions() const;

    Cost cost(State const& s, Action const& a) const;

    Cost cost(Action const& a) const;

    PrState successors_prob(State const& s, Action const& a) const;

    std::set<State> successors(State const& s, Action const& a) const;

    /*
     * SAS+ specific methods to handle variables and the goal formula
     */
    size_t numVariables() const;

    size_t numMaxEffects() const;

    SasPlusVariable variable(size_t i) const;

    /// Use this method in a range-based for loop to iterate over all variables
    std::vector<SasPlusVariable> const& variables() const;

    SasPlusPartialState const& goal() const;

    std::string to_string(SasPlusPartialState const& s) const;

    std::string to_string(State const& s) const;

    std::string to_string(Action const& a) const;

    void print(SasPlusPartialState const& s) const;

    void print(State const& s) const;

    void print(Action const& a) const;

    std::string get_name(Action const& a) const;

    std::string const& name() const;

    /// Print tireworld state
    void print_state_tri(State const& s) {
//      std::cout<<s<<std::endl;
//      for (size_t i = 0; i < numVariables(); i++) {
//        if (idx_to_name_[i].find("vehicle-at") != std::string::npos ||
//            idx_to_name_[i].find("flattire") != std::string::npos ||
//            idx_to_name_[i].find("spare-in") != std::string::npos) {
//        }
//        std::cout<<idx_to_name_[i]<<" "<<s[i]<<std::endl;
//      }
    }

    SasPlusMOSSPDet determinise();

   private:
    std::string name_;
    State s0_;
    SasPlusPartialState goal_;
    std::vector<Action> actions_;
    std::vector<SasPlusVariable> variables_;
    size_t max_effects_;
  };

} // namespace SasPlus

// Placing this here because SasPlusState is a std::vector so operator<< needs
// to be either in the current namespace or the std namespace
inline std::ostream& operator<<(std::ostream& os,
                                SasPlus::SasPlusState const& state) {
  for (bool first_item = true; auto const& value : state) {
    os << (first_item ? first_item = false, "[" : " ") << value;
  }
  os << "]";
  return os;
}

// See  operator<<(std::ostream& os, SasPlus::SasPlusState) for placement
// explanation
inline std::ostream& operator<<(
  std::ostream& os, SasPlus::SasPlusPartialState const& partial_state) {
  for (bool first_item = true; auto const& idx_value : partial_state) {
    os << (first_item ? first_item = false, "[" : " ") << "{v_"
       << idx_value.first << " = " << idx_value.second << "}";
  }
  os << "]";
  return os;
}

#endif // SRC_REPRESENTATIONS_SASPLUS_H_
