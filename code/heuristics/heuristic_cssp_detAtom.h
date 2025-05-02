#ifndef HEURISTIC_CSSP_DETATOM_H
#define HEURISTIC_CSSP_DETATOM_H

#include <algorithm>  // max, min
#include <list>
#include <vector>
#include <functional>

#include "heuristic_cssp_iface.h"

#include "../ext/mgpt/actions.h"
#include "../ext/mgpt/global.h"
#include "../ext/mgpt/states.h"
#include "../ext/mgpt/problems.h"


#if not defined VEC_DET_ATOM_MAX_ACTIONS
#define VEC_DET_ATOM_MAX_ACTIONS 8192
#endif


/*
 * This is a generalization of determinizationBasedAtomHeuristic to Constrained
 * SSPs for the constrained functions.
 */
class VectorDetermBasedAtomHeuristic : public TimedHeuristicCSSPIface
{
 public:
  VectorDetermBasedAtomHeuristic(std::string const& name,
                                 problem_t const& problem);

  virtual ~VectorDetermBasedAtomHeuristic() { statistics(); }

  virtual std::string name() const override { return name_; }

 protected:
  // * Compute the cost of the set of atoms using atom_cost. If this
  //   function returns the:
  //   - sum of all costs, then H_add is obtained
  //   - max of all costs, then H_max is obtained
  virtual double costSetOfAtoms(atomList_t const& atoms,
                                std::vector<double> const& atom_cost) const = 0;

 private:  // methods

  // from HeuristicCSSPIface
  void computeValueVec(state_t const& s, std::vector<double>& h_vec) override {
    assert(h_vec.size() == problem_.numCostFunctions());
    if (atom_cost_.size() == 0)
      return;

    if (relaxation_->goalT().holds(s, relaxation_->nprec())) {
      for (size_t i = 0; i < h_vec.size(); i++) { h_vec[i] = 0.0; }
    }
    else {
      computeCostOfAtoms(s);
      for (size_t i = 0; i < h_vec.size(); i++) {
        h_vec[i] = std::min(dead_end_vector_[i],
                            costSetOfAtoms(relaxation_->goalT().atom_list(0),
                                           atom_cost_[i]));
      }
    }
  }

  // - Compute the cost of each atom from state s using the h_1 definition.
  // - The values for the cost function associated with the i-th constraint are
  //   stored in atom_cost_[i] and action_cost_[i]
  void computeCostOfAtoms(state_t const& s);


 private:  // members
  std::string name_;
  problem_t const& problem_;

  // Using  gpt::__strong_relaxation instead
  problem_t const* relaxation_;
//  const problem_t &relaxation_; // strong relaxation, i.e., result of removing
//                                // the deletes from the STRIPS representation
//                                // of the all-outcomes determinization of the
//                                // original problem

  // Size: # of cost functions
  std::vector<double> dead_end_vector_;

  // Size: # of cost functions TIMES # of atoms
  // Position k,i: the cost of making atom(i) true in the current state
  // according to the k-th cost function
  std::vector<std::vector<double>> atom_cost_;

  // Size: # of cost functions TIMES operator_ptr_.size()
  // Position k,i: equivalent to costSetOfAtoms(operator_ptr_[i], atom_cost_[k])
  //
  // - this is used for speed up purposes (caching)
  std::vector<std::vector<double>> action_cost_;

  // has_prec_[i] is the list of indexes of operator_ptr_ such that, for all
  // j \in has_prec_[i], atom(i) \in precondition(operator_ptr_[j])
  std::vector<std::list<size_t>> has_prec_;

  // List of deterministic actions in the all-outcome determinization.
  std::vector<const deterministicAction_t*> operator_ptr_;
};


/*
 * FWT: instead of making VectorDetermBasedAtomHeuristic a template and have
 * all its implementation in the header, I decided to create this template as an
 * intermediary step to simplify extending VectorDetermBasedAtomHeuristic to
 * h-max and h-add
 */
template<char const* nameArg, typename CostSetOfAtomsFunctor>
class VectorDetermBasedAtomHeuristicTemplate
                                         : public VectorDetermBasedAtomHeuristic
{
 public:
  VectorDetermBasedAtomHeuristicTemplate(problem_t const& problem)
    : VectorDetermBasedAtomHeuristic(nameArg, problem)
  { }
 private:
  double costSetOfAtoms(atomList_t const& atoms,
                          std::vector<double> const& atom_cost) const override {
    return functor_(atoms, atom_cost);
  }
  CostSetOfAtomsFunctor functor_;
};



/*
 * H-Max Vectorized
 */
struct MaxCostSetOfAtomsVec {
  double operator()(atomList_t const& atoms,
                    std::vector<double> const& atom_cost) const
  {
    double max = 0;
    for (size_t i = 0; i < atoms.size(); ++i)
      max = std::max(max, atom_cost[atoms.atom(i)]);
    return max;
  }
};
static constexpr const char __h_max_vec_name[] = "h-max-vec";
using VectorHMax = VectorDetermBasedAtomHeuristicTemplate<__h_max_vec_name,
                                                          MaxCostSetOfAtomsVec>;


/*
 * H-Add Vectorized
 */
struct SumCostSetOfAtomsVec {
  double operator()(atomList_t const& atoms,
                    std::vector<double> const& atom_cost) const
  {
    double sum = 0;
    for (size_t i = 0; i < atoms.size(); ++i)
      sum += atom_cost[atoms.atom(i)];
    return sum;
  }
};
static constexpr const char __h_add_vec_name[] = "h-add-vec";
using VectorHAdd = VectorDetermBasedAtomHeuristicTemplate<__h_add_vec_name,
                                                          SumCostSetOfAtomsVec>;

#endif  // HEURISTIC_CSSP_DETATOM_H
