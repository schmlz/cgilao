#ifndef HEURISTIC_CSSP_IFACE_H
#define HEURISTIC_CSSP_IFACE_H

#include <iostream>
#include <vector>
#include <memory>

#include "h_add.h"
#include "h_max.h"
#include "lm_cut.h"

#include "../ext/mgpt/problems.h"
#include "../ext/mgpt/states.h"
#include "../utils/die.h"


class HeuristicCSSPIface;
using HeuristicCSSPUniqPtr = std::unique_ptr<HeuristicCSSPIface>;


/*******************************************************************************
 *
 * Heuristic for a Constrained SSP.
 *
 * The difference from heuristic_t is that it returns a vector with one value
 * for each cost function of the C-SSP
 *
 *
 ******************************************************************************/
class HeuristicCSSPIface {
 public:
  HeuristicCSSPIface() { }
  virtual ~HeuristicCSSPIface() { }

  virtual std::string name() const = 0;

  // Populates h_vec with the heuristic value for each cost function
  virtual void valueVec(state_t const& s, std::vector<double>& h_vec) = 0;
};


class MainCostOnlyHeuristic : public HeuristicCSSPIface {
 public:
  // CTOR that uses a unique_ptr and gets the ownership of h
  MainCostOnlyHeuristic(ConstrSSPIface const& cssp, std::unique_ptr<heuristic_t> h)
    : HeuristicCSSPIface(), name_("main-only-" + h->name()),
      cssp_(cssp), dead_end_vector_(cssp.numCostFunctions(), -1.0),
      h_uptr_(std::move(h)), h_(h_uptr_.get())
  {
    initDeadendVec();
  }

  // Legacy CTOR that receives a regular pointer and just uses it (do not own the
  // pointer)
  MainCostOnlyHeuristic(ConstrSSPIface const& cssp, heuristic_t* h)
    : HeuristicCSSPIface(), name_("main-only-" + h->name()),
      cssp_(cssp), dead_end_vector_(cssp.numCostFunctions(), -1.0),
      h_uptr_(nullptr), h_(h)
  {
    initDeadendVec();
  }


  ~MainCostOnlyHeuristic() { }

  std::string name() const override { return name_; }

 protected:
  /*
   * HeuristicCSSPIface Interface
   */
  void valueVec(state_t const& s, std::vector<double>& h_vec) override {
    if (!cssp_.hasApplicableActions(s)) {
      h_vec = dead_end_vector_;
    }
    else {
      for (double& h_i : h_vec)
        h_i = 0.0;
      h_vec[ACTION_COST] = h_->value(s);
    }
  }

 private:
  void initDeadendVec() {
    dead_end_vector_[ACTION_COST] = gpt::dead_end_value.double_value();
    ConstraintMap const& cost_constraints = cssp_.constraints();
    for (size_t i = 0; i < cost_constraints.size(); ++i) {
      dead_end_vector_[cost_constraints.costIdx(i)] = cost_constraints.deadendPenalty(i);
    }
  }

  std::string name_;
  ConstrSSPIface const& cssp_;
  std::vector<double> dead_end_vector_;
  std::unique_ptr<heuristic_t> h_uptr_;
  // TODO: Here for legacy reasons
  heuristic_t* h_;
};



class TimedHeuristicCSSPIface : public HeuristicCSSPIface {
 public:
  TimedHeuristicCSSPIface()
    : total_calls_(0), mean_cputime_(0.0), m2_cputime_(0.0)
  { }

  virtual ~TimedHeuristicCSSPIface() { }

  void statistics() const {
    std::cout << "[" << name() << " heuristic]: total calls = "
              << total_calls_ << std::endl
              << "[" << name() << " heuristic]: 95CI cputime (in secs) = ";
    printf("%0.7f -+ %0.7f\n", mean_cputime_,
            (1.96 * sqrt(m2_cputime_ / (total_calls_ * (total_calls_-1)))));
  }

  /*
   * Non-virtual wrapper to computeValueVec. This wrapper allow us to compute
   * on-the-fly stats about the heuristic and help debugging it.
   */
  void valueVec(state_t const& s, std::vector<double>& h_vec) override {
    START_TIMING("heuristic_value");
    uint64_t before = get_cputime_usec();
    computeValueVec(s, h_vec);
    double time_spent_in_s = (get_cputime_usec() - before) / (double) 1000000;
    double delta = time_spent_in_s - mean_cputime_;
    total_calls_++;
    mean_cputime_ += delta/total_calls_;
    m2_cputime_ += delta * (time_spent_in_s - mean_cputime_);
    STOP_TIMING("heuristic_value");
  }

 protected:
  /*
   * Heuristic main method
   *
   * The vector has size problem.constraints().size() and the vector[i] is the
   * heuristic for the i-th constraint, i.e., for the cost function 
   * problem.constraints().costIdx(i)
   */
  virtual void computeValueVec(state_t const& s, std::vector<double>& h_vec) = 0;

 private:
  /*
   * Private member variables
   */
  size_t total_calls_;
  double mean_cputime_;
  double m2_cputime_;
};


class SmartZeroVector : public TimedHeuristicCSSPIface {
 public:
  SmartZeroVector(ConstrSSPIface const& cssp)
    : TimedHeuristicCSSPIface(), name_("H-vec{smart-zero}"), cssp_(cssp),
      dead_end_vector_(cssp.numCostFunctions(), -1.0)
  {
    dead_end_vector_[ACTION_COST] = gpt::dead_end_value.double_value();
    ConstraintMap const& cost_constraints = cssp.constraints();
    for (size_t i = 0; i < cost_constraints.size(); ++i) {
      dead_end_vector_[cost_constraints.costIdx(i)] = cost_constraints.deadendPenalty(i);
    }
  }

 ~SmartZeroVector() { statistics(); }

  std::string name() const override { return name_; }

 protected:
  /*
   * HeuristicCSSPIface Interface
   */
  void computeValueVec(state_t const& s, std::vector<double>& h_vec) override {
    if (!cssp_.hasApplicableActions(s)) {
      h_vec = dead_end_vector_;
    }
    else {
      for (double& h_i : h_vec)
        h_i = 0.0;
    }
  }

 private:
  std::string name_;
  ConstrSSPIface const& cssp_;
  std::vector<double> dead_end_vector_;
};


using VecUniqPtrHeuristic = std::vector<std::unique_ptr<heuristic_t>>;

template<char const* nameTemplate, class HeuristicClass>
class VectorIndependentHeuristics : public TimedHeuristicCSSPIface {
 public:
  VectorIndependentHeuristics(problem_t const& problem)
    : TimedHeuristicCSSPIface(), name_(nameTemplate)
  {
    size_ = problem.domain().functions().size();
    for (size_t i = 0; i < size_; ++i) {
      heuristics_.push_back(std::unique_ptr<heuristic_t>(new HeuristicClass(problem, i)));
    }
  }

  ~VectorIndependentHeuristics() { statistics(); }

  std::string name() const override { return name_; }

 private:
  /*
   * HeuristicCSSPIface Interface
   */
  void computeValueVec(state_t const& s, std::vector<double>& h_vec) override {
    assert(h_vec.size() == size_);
    for (size_t i = 0; i < size_; ++i)
      h_vec[i] = heuristics_[i]->value(s);
  }

  std::string name_;
  VecUniqPtrHeuristic heuristics_;
  size_t size_;
};


static constexpr const char __h_vec_max_det_name[] = "h-vec{h-max-det}";
using VectorIndepHMax = VectorIndependentHeuristics<__h_vec_max_det_name,
                                                    HMaxAllOutcomesDet>;

static constexpr const char __h_vec_add_det_name[] = "h-vec{h-add-det}";
using VectorIndepHAdd = VectorIndependentHeuristics<__h_vec_add_det_name,
                                                    HAddAllOutcomesDet>;

static constexpr const char __h_vec_lmcut_det_name[] = "h-vec{h-lmcut-det}";
using VectorIndepLMCut = VectorIndependentHeuristics<__h_vec_lmcut_det_name,
                                                     LMCutHeuristic>;


template<typename HeuristicClass, typename NameFunctor>
class VectorSasPlusHeuristicsTemplate : public TimedHeuristicCSSPIface {
 public:
  VectorSasPlusHeuristicsTemplate(problem_t const& problem, bool use_deadend_transformation)
    : TimedHeuristicCSSPIface()
  {
    size_ = problem.domain().functions().size();
    for (size_t i = 0; i < size_; ++i) {
      // TODO: EFFICIENCY: FIXME: add a constructor for CostConstrROC (i.e.,
      // OperatorCountTemplate) that receives a HackedPrSasProblem as parameter,
      // so that we don't have to translate the problem for each of the
      // CostConstrROC heuristics
      heuristics_.push_back(std::unique_ptr<HeuristicClass>(new
                       HeuristicClass(problem, use_deadend_transformation, i)));
    }
  }

  ~VectorSasPlusHeuristicsTemplate() { statistics(); }

  std::string name() const override { return NameFunctor::name(); }

 private:
  /*
   * HeuristicCSSPIface Interface
   */
  void computeValueVec(state_t const& s, std::vector<double>& h_vec) override {
    assert(h_vec.size() == size_);
    for (size_t i = 0; i < size_; ++i)
      h_vec[i] = heuristics_[i]->value(s);
  }

  std::vector<std::unique_ptr<HeuristicClass>> heuristics_;
  size_t size_;
};



// This class was originally designed for PlannerIDualColGen (i.e., i-dual with real column
// generation) because the heuristic might be called several times for the same state (there
// is no value function to be initialized with the heuristic as default).
class CachedVectorHeuristicWrapper {
 using VecDoubles = std::vector<double>;
 using HashStateVecDoubles = std::unordered_map<state_t, VecDoubles, hashState>;

 public:
  CachedVectorHeuristicWrapper(size_t num_cost_funcs, HeuristicCSSPUniqPtr h_vector)
    : num_cost_funcs_(num_cost_funcs), total_calls_(0), total_cached_calls_(0),
      h_vector_(std::move(h_vector))
  {
    assert(num_cost_funcs_ > 0);
  }

  ~CachedVectorHeuristicWrapper() {
    std::cout << "[CachedVectorHeuristicWrapper]: total calls: " << total_calls_ << "\n"
              << "[CachedVectorHeuristicWrapper]: cached call: " << total_cached_calls_
              << " (ratio: " << ((double) total_cached_calls_ / total_calls_)<< ")" << std::endl;
  }

  VecDoubles const& getOrCompute(state_t const& s) {
    total_calls_++;
    auto it = cache_.find(s);
    if (it != cache_.end()) {
      total_cached_calls_++;
      return it->second;
    }
    VecDoubles& h_vals = cache_[s];
    h_vals.resize(num_cost_funcs_, 0.0);
    h_vector_->valueVec(s, h_vals);
    return h_vals;
  }

  std::string name() const {
    return "Cached(" + h_vector_->name() + ")";
  }

  size_t totalUniqueCalls() const { return total_calls_ - total_cached_calls_; }

  // TODO: if memory becomes an issue, we can add the following method
  //    void getAndDropCache(state_t const& s, VecDoubles& h_vals) { }
  // that would populate h_vals and if the values was on the cache, then remove it. This method
  // would be called whenever we know the heuristic for a given state is not necessary anymore for
  // the computation of the reduced cost.
 private:
  size_t num_cost_funcs_;
  size_t total_calls_;
  size_t total_cached_calls_;
  HeuristicCSSPUniqPtr h_vector_;
  HashStateVecDoubles cache_;
};

#endif  // HEURISTIC_CSSP_IFACE_H
