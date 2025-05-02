#include <bitset>
#include <limits>

#include "heuristic_cssp_detAtom.h"

/*******************************************************************************
 *
 * VectorDetermBasedAtomHeuristic: base for h_add, h_max and similar for
 * probabilistic problems.
 *
 ******************************************************************************/
VectorDetermBasedAtomHeuristic::VectorDetermBasedAtomHeuristic(
    std::string const& name, problem_t const& problem)
  : TimedHeuristicCSSPIface(), name_(name), problem_(problem),
    relaxation_(nullptr),
    dead_end_vector_(problem.numCostFunctions(), std::numeric_limits<double>::max()),
    atom_cost_(problem.numCostFunctions()),
    action_cost_(problem.numCostFunctions()),
    has_prec_(problem.number_atoms())
    //, relaxation_(problem.strong_relaxation(use_self_loop_relaxation))
{
  if (!gpt::__strong_relaxation) {
    gpt::__strong_relaxation = &problem.strong_relaxation();
  }
  relaxation_ = gpt::__strong_relaxation;

  for (size_t i = 0; i < problem.numCostFunctions(); ++i) {
    atom_cost_[i].resize(problem.number_atoms());
    action_cost_[i].resize(relaxation_->actionsT().size());
  }

  dead_end_vector_[ACTION_COST] = gpt::dead_end_value.double_value();
  ConstraintMap const& cost_constraints = problem.constraints();
  for (size_t i = 0; i < cost_constraints.size(); ++i) {
    dead_end_vector_[cost_constraints.costIdx(i)] = cost_constraints.deadendPenalty(i);
  }

  actionList_t::const_iterator it;

  _D(DEBUG_H_DET, std::cout << "Actions in the strong relaxation\n";)
  for (it = relaxation_->actionsT().begin();
       it != relaxation_->actionsT().end(); ++it)
  {
    deterministicAction_t const* dact =
                                dynamic_cast<deterministicAction_t const*>(*it);
    _D(DEBUG_H_DET, std::cout << "====================" << std::endl;
          dact->print_full();
          std::cout << std::endl;)
    operator_ptr_.push_back(dact);
  }

  if (operator_ptr_.size() > VEC_DET_ATOM_MAX_ACTIONS) {
    std::cout << "ERROR! VEC_DET_ATOM_MAX_ACTIONS is too small. Currently its "
              << "value is " << VEC_DET_ATOM_MAX_ACTIONS << " and it should be "
              << "at least " << operator_ptr_.size() << ". Quitting."
              << std::endl;
    exit(240);
  }

  // For each action o in the strong relaxation
  for (size_t o = 0; o < operator_ptr_.size(); ++o) {
    // For each atom i in the precondition of o
    FANCY_DIE_IF(operator_ptr_[o]->precondition().size() != 1, 171,
        "Action %s has a (disjuntive) precondition of size %d",
        operator_ptr_[o]->name(),
        operator_ptr_[o]->precondition().atom_list(0).size());

    for (size_t i = 0;
         i < operator_ptr_[o]->precondition().atom_list(0).size(); ++i)
    {
      ushort_t atom = operator_ptr_[o]->precondition().atom_list(0).atom(i);
      has_prec_[atom].push_back(o);
    }
  }
  assert(relaxation_->goalT().size() == 1);
}


void VectorDetermBasedAtomHeuristic::computeCostOfAtoms(state_t const& s) {

  assert(!relaxation_->goalT().holds(s, relaxation_->nprec()));

  // Offset. If negative atoms are represented, then positive atoms are the
  // even values of i.
  ushort_t inc = relaxation_->nprec() ? 1 : 2;
  _D(DEBUG_H_DET, std::cout << "inc == " << inc << std::endl;)

  // Position i: true if the cost of any atom in the precondition of
  //             operator_ptr_[i] has decreased.
  static std::bitset<VEC_DET_ATOM_MAX_ACTIONS> aa;
  aa.reset();

  for (size_t i = 0; i < operator_ptr_.size(); i++) {
    if (operator_ptr_[i]->precondition().atom_list(0).size() == 0) {
      // This operator has no precondition, so it is already applicable and
      // needs to be considered
      aa[i] = true;
    }

    for (auto& action_cost_K : action_cost_) {
      action_cost_K[i] = std::numeric_limits<double>::max();
    }
  }

  for (atom_t i = 0; i < problem_t::number_atoms(); i += inc) {
    // For some reason s.holds(i) is not returning true for the negation of
    // an atom. Probably it is related to the fact that the s is a state of
    // the original problem, not the relaxation.
    // If
    //  - inc equals one, we should also set the negative atoms.
    //  - i is odd, then i represent (not i-1)
    //  - !s.holds(i-1)
    // then
    //  - i-1 should hold in s.
    if (s.holds(i) || (inc == 1 && i % 2 == 1 && !s.holds(i-1))) {
//      _D(DEBUG_H_DET, std::cout << "  atom i=" << i
//                                << " holds in s, so atom_cost_[i] = 0\n";
//      )
      for (auto& atom_cost_K : atom_cost_) {
        atom_cost_K[i] = 0.0;
      }
      for(std::list<size_t>::iterator it = has_prec_[i].begin();
          it != has_prec_[i].end(); it++)
      {
        _D(DEBUG_H_DET, std::cout << "    make aa true for operator "
                                   << *it << std::endl;)
        aa[*it] = true;
      }
    }
    else {
      for (auto& atom_cost_K : atom_cost_) {
        atom_cost_K[i] = std::numeric_limits<double>::max();
      }
    }
  }

  /*
   * Computing atom_cost_ and action_cost_ for the given state
   */
  bool done = false;
  while (!done) {
    done = true;  // Assuming we already converged
    for (size_t i = 0; i < operator_ptr_.size(); i++) {
      if (!aa[i]) {
        _D(DEBUG_H_DET, std::cout << "Operator_ptr[" << i << "] = "
                                  << operator_ptr_[i]->name()
                                  << " has aa false :(" << std::endl;)
        continue;
      }

      _D(DEBUG_H_DET, std::cout << "Processing operator_ptr[" << i << "] = "
                  << operator_ptr_[i]->name()
                  << " (its aa was true)" << std::endl;)
      aa[i] = false;  // i needed to be updated and we're doing it now

      bool finite_prec_cost =  true;
      atomList_t const& prec = operator_ptr_[i]->precondition().atom_list(0);

      // Searching for an atom in prec(i) that has infinity cost, i.e.,
      // double::max. This only happens when the preconditions of the operator
      // is not reachable; therefore, it is INDEPENDENT of the cost function
      for (size_t j = 0; j < prec.size() && finite_prec_cost; ++j) {
        if (atom_cost_[0][prec.atom(j)] == std::numeric_limits<double>::max()) {
          _D(DEBUG_H_DET, std::cout << "  One of its preconditions still "
                                    << "cost infinity, so giving up\n";)
          finite_prec_cost = false;
        }
      }

      if (!finite_prec_cost) {
        _D(DEBUG_H_DET, std::cout << "Operator_ptr[" << i << "] = "
                    << operator_ptr_[i]->name()
                    << " is NOT applicable :(" << std::endl;)
        continue;
      }

      // Every atom in the precondition of operator_ptr_[i] costs less than
      // infinity (double::max), so we can derive a plan in which the action is
      // applicable
      _D(DEBUG_H_DET, std::cout << "  Operator_ptr[" << i << "] = "
                                << operator_ptr_[i]->name()
                                << " is applicable!" << std::endl;
        operator_ptr_[i]->print_full();
        std::cout << std::endl;
      )

      atomList_t const& add_list =
                               operator_ptr_[i]->effect().s_effect().add_list();

      // Now we look at each cost function associated to a constraint
      // independently
      VecRationals const vec_cost_a = operator_ptr_[i]->costVector();
      for (size_t k = 0; k < vec_cost_a.size(); ++k) {
        double prec_cost_K = costSetOfAtoms(prec, atom_cost_[k]);

        if (e_less((prec_cost_K), (action_cost_[k][i]))) {
          done = false;  // The cost of action operator_ptr_[i] decreased
                         // by more than epsilon, so we didn't converge.
          _D(DEBUG_H_DET, std::cout << "  Cost to make action applicable "
                      << "decreased by " << action_cost_[i] - cost
                      << " to " << cost << std::endl;)
          /* set new cost for applying action */
          action_cost_[k][i] = prec_cost_K;
          /* cost of reaching each atom in the add list, i.e., cost of the
           * preconditions of the current operator plus the cost of the
           * operator itself */
          double cost_after_exec_i = prec_cost_K
                                       + vec_cost_a[k].double_value();


          // Updating, if necessary, the cost of the atoms added by
          // operator_ptr_[i], the actions supporting the added atom and the
          // actions that need to be updated (vector aa)
          for (size_t ia = 0; ia < add_list.size(); ia++) {
            atom_t a = add_list.atom(ia);
            if (e_less((cost_after_exec_i), (atom_cost_[k][a]))) {
              _D(DEBUG_H_DET, std::cout << "  Cost to make atom " << a
                          << " true in " << k "-th cost func decreased by "
                          << atom_cost_[k][a] - cost_after_exec_i
                          << " to " << cost_after_exec_i << std::endl;)
              atom_cost_[k][a] = cost_after_exec_i;
              for (std::list<size_t>::iterator it = has_prec_[a].begin();
                  it != has_prec_[a].end(); it++)
              {
                aa[*it] = true;
              }
            }  // if cost of atom a decreased
          }  // for each atom a in add list of operator_ptr_[i]
        }  // if action_cost_[i] - cost > epsilon
      }  // for each constraint k
    }  // for each action o
  }  // while !done
}
