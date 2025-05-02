#ifndef POLICY_H
#define POLICY_H

#include <iostream>
#include <vector>
#include <unordered_map>

#include "../ext/mgpt/actions.h"
#include "../utils/die.h"
#include "../ext/mgpt/states.h"
#include "../utils/iterator_wrapper.h"
#include "ssp_utils.h"

/******************************************************************************
 *
 * PolicyIface
 *
 * This interface represents a stationary Markovian policy. This policy can be
 * deterministic or probabilistic
 *
 *****************************************************************************/
class PolicyIface {
 public:
  virtual action_t const* action(state_t const& s) const = 0;
  virtual bool isDefinedFor(state_t const& s) const = 0;
  // Returns true if pi(s) existed before the call
  virtual bool unset(state_t const& s) = 0;
  virtual void clear() = 0;
};



/******************************************************************************
 *
 * DetPolicyIface
 *
 * This interface represents a DETERMINISTIC stationary Markovian policy.
 *
 * Bellow are also some of its helper classes
 *
 *****************************************************************************/
// Helper class for the iterator of DetPolicyIface
class StateActionPair {
 public:
  StateActionPair(state_t const& s, action_t const* a) : s_(s), a_(a) { }
  state_t const& state() const { return s_; }
  action_t const* action() const { return a_; }
 private:
  state_t const& s_;
  action_t const* a_;
};


// Typedefs from iterator_wrapper for in-house type erasure solution
using StateActionPairConstIteIfaceUniqPtr =
                              ConstIteratorIfaceUniqPtr<StateActionPair const>;
using StateActionPairConstIteIface = ConstIteratorIface<StateActionPair const>;
using StateActionPairConstIteWrapper =
                                   ConstIteratorWrapper<StateActionPair const>;



class DetPolicyIface : public PolicyIface {
 public:

  /*
   * PolicyIface Interface
   */
  virtual action_t const* action(state_t const& s) const = 0;
  // Returns true if pi(s) is defined for the state s
  virtual bool isDefinedFor(state_t const& s) const = 0;

  // Returns true if pi(s) existed before the call
  virtual bool unset(state_t const& s) = 0;
  virtual void clear() = 0;

  /*
   * Methods for deterministic policies
   */
  virtual void set(state_t const& s, action_t const* a) = 0;

  // Iterators. This allows to write for (auto const& pair : pi) { }
  virtual StateActionPairConstIteWrapper begin() const = 0;
  virtual StateActionPairConstIteWrapper end() const = 0;
};


/*******************************************************************************
 * Default Implementation
 ******************************************************************************/
class HashMapDetPolicy;
using DetPolicy = HashMapDetPolicy;


/*******************************************************************************
 * Implementations of the DetPolicyIface
 ******************************************************************************/

// This typedef is here to be used in both the policy and iterator
// implementations
using HashStateActionP = HashMapState<action_t const*>;

/*
 * HashMapStateActionPairConstIte
 */
class HashMapStateActionPairConstIte : public StateActionPairConstIteIface {
 public:
  HashMapStateActionPairConstIte(HashStateActionP::const_iterator it,
      HashStateActionP::const_iterator end)
    : it_(it), end_(end)
  { }

  ~HashMapStateActionPairConstIte() { }

  HashMapStateActionPairConstIte& operator++() { ++it_; return *this; }

  bool operator!=(HashMapStateActionPairConstIte const& rhs) const {
    return it_ != rhs.it_ || end_ != rhs.end_;
  }

  bool operator!=(StateActionPairConstIteIfaceUniqPtr const& rhs) const {
    // Hack for efficiency: nullptr represents the end of the range
    if (!rhs)  return it_ != end_;

    HashMapStateActionPairConstIte const* conv_rhs =
                dynamic_cast<HashMapStateActionPairConstIte const*>(rhs.get());
    if (conv_rhs) {
      return operator!=(*conv_rhs);
    }
    else {
      return true;
    }
  }

  StateActionPair const operator*() const {
    return StateActionPair(it_->first, it_->second);
  }

  StateActionPairConstIteIfaceUniqPtr clone() const {
    return StateActionPairConstIteIfaceUniqPtr(
                                new HashMapStateActionPairConstIte(it_, end_));
  }

 private:
  HashStateActionP::const_iterator it_, end_;
};


/*
 * HashMapDetPolicy
 */
class HashMapDetPolicy : public DetPolicyIface {
 public:
  HashMapDetPolicy() { }
  virtual ~HashMapDetPolicy() { }

  /*
   * PolicyIface Interface
   */
  action_t const* action(state_t const& s) const override {
    auto const it = hash_.find(s);
    if (it == hash_.end()) {
      return NULL;
    }
    return it->second;
  }

  bool isDefinedFor(state_t const& s) const override {
    return hash_.find(s) != hash_.end();
  }

  bool unset(state_t const& s) override { return hash_.erase(s); }
  void clear() override { hash_.clear(); }


  /*
   * DetPolicyIface Interface
   */
  void set(state_t const& s, action_t const* a) override { hash_[s] = a; }

  StateActionPairConstIteWrapper begin() const override {
    return StateActionPairConstIteWrapper(StateActionPairConstIteIfaceUniqPtr(
          new HashMapStateActionPairConstIte(hash_.begin(), hash_.end())));
  }

  StateActionPairConstIteWrapper end() const override {
    return StateActionPairConstIteWrapper(StateActionPairConstIteIfaceUniqPtr(
                                                                      nullptr));
  }

 private:
  /*
   * Member variables
   */
  HashStateActionP hash_;
};

inline std::ostream& operator<<(std::ostream& os, DetPolicyIface const& pi) {
  for (auto const& pair : pi) {
    os << pair.state().toStringFull(gpt::problem, false, true) << ": "
      << (pair.action() ? pair.action()->name() : "NULL")
      << std::endl;
  }
  return os;
}



/******************************************************************************
 *
 * ProbPolicy
 *
 * This is an implementation of a PROBABILISTIC stationary Markovian policy.
 *
 *****************************************************************************/
using DistActionP = ProbDistVector<action_t const*>;
using HashStateToDistActionP = HashMapState<DistActionP>;

class ProbPolicy : public PolicyIface {
 public:
  /*
   * PolicyIface Interface 
   */
  // For probabilistic policies, this method should sample an action and return
  // it.
  action_t const* action(state_t const& s) const {
    auto it = prob_dist_.find(s);
    if (it == prob_dist_.end()) {
      return nullptr;
    }
    // computing the normalizing constant before sampling
    return it->second.sample(-1);
  }

  // Returns true if pi(s) is defined for the state s
  bool isDefinedFor(state_t const& s) const {
    return prob_dist_.find(s) != prob_dist_.end();
  }

  // Returns true if pi(s) existed before the call
  bool unset(state_t const& s) {
    auto it = prob_dist_.find(s);
    if (it == prob_dist_.end()) {
      return false;
    }
    prob_dist_.erase(it);
    return true;
  }

  void clear() { prob_dist_.clear(); }


  /*
   * Methods for probabilistic policies
   */
  void set(state_t const& s, DistActionP const& prob_dist) {
    prob_dist_[s] = prob_dist;
  }

  DistActionP& operator[](state_t const& s) {
    return prob_dist_[s];
  }

  HashStateToDistActionP::const_iterator begin() const {
    return prob_dist_.begin();
  }

  HashStateToDistActionP::const_iterator end() const {
    return prob_dist_.end();
  }

 private:
  HashStateToDistActionP prob_dist_;
};


inline std::ostream& operator<<(std::ostream& os, ProbPolicy const& pi) {
  for (auto const& it : pi) {
    os << it.first.toStringFull(gpt::problem, false, true) << ": {";
    DistActionP const& prob_pi = it.second;
    double normalizing_constant = prob_pi.normalizingConstant();
    for (auto const& pair : prob_pi) {
      action_t const* a = pair.event();
      os << (pair.prob() / normalizing_constant) << " "
         << (a ? a->name() : "NULL") << ", ";
    }
    os << "}" << std::endl;
  }
  return os;
}

void fancyProbPolicyDebug(ProbPolicy& prob_pi, state_t const& s0);
void fancyPartialProbPolicyDebug(ProbPolicy& prob_pi, state_t const& s0);



#endif  // POLICY_H
