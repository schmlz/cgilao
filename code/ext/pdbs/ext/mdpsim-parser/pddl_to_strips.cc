#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <map>

#include "pddl_to_strips.h"

#include "problems.h"
#include "det_pddl_builder.h"

#include "../../representations/strips.h"


using STRIPS::Mask;
using STRIPS::StripsAction;


namespace PPDDL_PDB {

StripsMOSSP parseToStrips(std::string const& domain_and_problem_fname) {
  MdpsimProblem const* problem = parsePPDDL(domain_and_problem_fname);
  assert(problem);
  return translateToStrips(*problem);
}


StripsMOSSP parseToStrips(std::string const& domain_fname,
                          std::string const& problem_fname) {
  MdpsimProblem const* problem = parsePPDDL(domain_fname, problem_fname);
  assert(problem);
  return translateToStrips(*problem);
}


// Translate state formula into mask for strips to be used as precondition and goal tests
Mask translateConjOfPosAtoms(StateFormula const& fml,
                             std::map<Atom const*, size_t> const& atom_to_idx) {
  if (fml.tautology()) {
    // A tautology for precondition and goal sake is a Mask of all zeros since the and of the any
    // state and the all-zero mask equals the all-zero mask. Another way of thinking about this is
    // that the all-zero mask requires no proposition to be true.
    return Mask(atom_to_idx.size());
  }

  // The all-ones Mask is **not** the equivalent of a contradiction
  assert(!fml.contradiction());

  if (Atom const* atm = dynamic_cast<Atom const*>(&fml)) {
    auto atom_ite = atom_to_idx.find(atm);
    assert(atom_ite != atom_to_idx.end());
    return Mask(atom_to_idx.size()).set(atom_ite->second);
  } else if (Conjunction const* conj = dynamic_cast<Conjunction const*>(&fml)) {
    Mask conj_mask(atom_to_idx.size());
    for (StateFormula const* const& conjunct : conj->conjuncts()) {
      assert(conjunct != nullptr);
      conj_mask |= translateConjOfPosAtoms(*conjunct, atom_to_idx);
    }
    return conj_mask;
  } else {
    std::cerr << "Formula " << fml << " is not supported in the goal\n";
    throw std::logic_error("Unsupported formula in the goal");
  }
  return Mask(0);
}


// Returns number of probabilistic effect outcomes (returns 1 if no probabilistic effects)
size_t getProbabilisticSize(Effect const& ppddl_effect) {
  using ConjuncEff = ConjunctiveEffect;
  using CondEff = ConditionalEffect;
  using ProbEff = ProbabilisticEffect;
  using QuantEff = QuantifiedEffect;

  if (ppddl_effect.empty()) {
    return 1;
  }

  if (ConjuncEff const* conj_eff = dynamic_cast<ConjuncEff const*>(&ppddl_effect)) {
    size_t ret = 1;
    for (Effect const* c : conj_eff->conjuncts()) {
      assert(c != nullptr);
      ret = std::max(ret, getProbabilisticSize(*c));
    }
    return ret;
  } else if (ProbEff const* prob_eff = dynamic_cast<ProbEff const*>(&ppddl_effect)) {
    return prob_eff->size();
  } else if ([[maybe_unused]] AddEffect const* add_eff = dynamic_cast<AddEffect const*>(&ppddl_effect)) {
    return 1;
  } else if ([[maybe_unused]] DeleteEffect const* del_eff = dynamic_cast<DeleteEffect const*>(&ppddl_effect)) {
    return 1;
  } else if (dynamic_cast<UpdateEffect const*>(&ppddl_effect)) {
    return 1;
  } else if (CondEff const* cond_eff = dynamic_cast<CondEff const*>(&ppddl_effect)) {
    std::cerr << "ConditionalEffect '" << *cond_eff << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  } else if (QuantEff const* quant_eff = dynamic_cast<QuantEff const*>(&ppddl_effect)) {
    std::cerr << "QuantifiedEffect '" << *quant_eff << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  } else {
    std::cerr << "Effect '" << ppddl_effect << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  }
}

// Translate ppddl effect from mdpsim parser to strips framework effect after probabilistic
// effects have been detected (see translateStripsEffect below)
void translateStripsEffectProb(Effect const& ppddl_effect,
                               STRIPS::Effect& strips_effect,
                               std::map<Atom const*, size_t> const& atom_to_idx)
{
  using ConjuncEff = ConjunctiveEffect;
  using CondEff = ConditionalEffect;
  using ProbEff = ProbabilisticEffect;
  using QuantEff = QuantifiedEffect;

  if (ppddl_effect.empty()) {
    // Current effect it a NO-OP so do nothing and return
    return;
  }

  if (AddEffect const* add_eff = dynamic_cast<AddEffect const*>(&ppddl_effect)) {
    Atom const& atom = add_eff->atom();
    auto const ite = atom_to_idx.find(&atom);
    assert(ite != atom_to_idx.end());
    strips_effect.add.set(ite->second);
  } else if (DeleteEffect const* del_eff = dynamic_cast<DeleteEffect const*>(&ppddl_effect)) {
    Atom const& atom = del_eff->atom();
    auto const ite = atom_to_idx.find(&atom);
    assert(ite != atom_to_idx.end());
    strips_effect.del.set(ite->second);
  } else if (ConjuncEff const* conj_eff = dynamic_cast<ConjuncEff const*>(&ppddl_effect)) {
    for (Effect const* c : conj_eff->conjuncts()) {
      assert(c != nullptr);
      translateStripsEffectProb(*c, strips_effect, atom_to_idx);
    }
  } else if (dynamic_cast<UpdateEffect const*>(&ppddl_effect)) {
    // DO NOT REMOVE. The supported update effects are handled by Action::cost().
  } else if (ProbEff const* prob_eff = dynamic_cast<ProbEff const*>(&ppddl_effect)) {
    std::cerr << "Nested probabilistic effect '" << *prob_eff << "' detected\n";
    throw std::logic_error("Unsupported effect");
  } else if (CondEff const* cond_eff = dynamic_cast<CondEff const*>(&ppddl_effect)) {
    std::cerr << "ConditionalEffect '" << *cond_eff << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  } else if (QuantEff const* quant_eff = dynamic_cast<QuantEff const*>(&ppddl_effect)) {
    std::cerr << "QuantifiedEffect '" << *quant_eff << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  } else {
    std::cerr << "Effect '" << ppddl_effect << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  }
}

// Translate ppddl effect with probabilities from mdpsim parser to strips framework effect
void translateStripsEffect(Effect const& ppddl_effect,
                           STRIPS::Effect& strips_effect,
                           STRIPS::PrEffect& strips_pr_effect,
                           STRIPS::PrDist& strips_pr_dist,
                           std::map<Atom const*, size_t> const& atom_to_idx)
{
  using ConjuncEff = ConjunctiveEffect;
  using CondEff = ConditionalEffect;
  using ProbEff = ProbabilisticEffect;
  using QuantEff = QuantifiedEffect;
  if (AddEffect const* add_eff = dynamic_cast<AddEffect const*>(&ppddl_effect)) {
    Atom const& atom = add_eff->atom();
    auto const ite = atom_to_idx.find(&atom);
    assert(ite != atom_to_idx.end());
    strips_effect.add.set(ite->second);
  } else if (DeleteEffect const* del_eff = dynamic_cast<DeleteEffect const*>(&ppddl_effect)) {
    Atom const& atom = del_eff->atom();
    auto const ite = atom_to_idx.find(&atom);
    assert(ite != atom_to_idx.end());
    strips_effect.del.set(ite->second);
  } else if (ConjuncEff const* conj_eff = dynamic_cast<ConjuncEff const*>(&ppddl_effect)) {
    for (Effect const* c : conj_eff->conjuncts()) {
      assert(c != nullptr);
      translateStripsEffect(*c, strips_effect, strips_pr_effect, strips_pr_dist, atom_to_idx);
    }
  } else if (ProbEff const* prob_eff = dynamic_cast<ProbEff const*>(&ppddl_effect)) {
    for (size_t i = 0; i < prob_eff->size(); i++) {
      Effect const& c = prob_eff->effect(i);
      translateStripsEffectProb(c, strips_pr_effect[i], atom_to_idx);
      strips_pr_dist[i] = double(prob_eff->probability(i));
    }
  } else if (dynamic_cast<UpdateEffect const*>(&ppddl_effect)) {
    // DO NOT REMOVE. The supported update effects are handled by Action::cost().
  } else if (CondEff const* cond_eff = dynamic_cast<CondEff const*>(&ppddl_effect)) {
    std::cerr << "ConditionalEffect '" << *cond_eff << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  } else if (QuantEff const* quant_eff = dynamic_cast<QuantEff const*>(&ppddl_effect)) {
    std::cerr << "QuantifiedEffect '" << *quant_eff << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  } else {
    std::cerr << "Effect '" << ppddl_effect << "' not supported\n";
    throw std::logic_error("Unsupported effect");
  }
}

// Translate mdpsim ppddl action into strips action
StripsAction translateAction(Action const& ppddl_action,
                             std::map<Atom const*, size_t> const& atom_to_idx,
                             std::map<std::string, size_t> const& metric_to_idx)
{
  // translate strips precondition
  Mask strips_prec = translateConjOfPosAtoms(ppddl_action.precondition(), atom_to_idx);

  // effect that applies regardless of probability
  STRIPS::Effect strips_effect(atom_to_idx.size());

  // initialise probabilistic effects and distribution
  size_t pr_size = getProbabilisticSize(ppddl_action.effect());
  STRIPS::PrDist strips_pr_dist(pr_size);  // probability distribution
  STRIPS::PrEffect strips_pr_effect;  // probabilistic effects
  for (size_t i = 0; i < pr_size; i++) {
    strips_pr_effect.push_back(STRIPS::Effect(strips_prec.size()));
  }


  // translate ppddl effects into strips effects
  translateStripsEffect(ppddl_action.effect(), strips_effect, strips_pr_effect, strips_pr_dist, atom_to_idx);
  if (pr_size == 1) {  // account for deterministic effects
    strips_pr_dist[0] = 1;
  }

  // combine strips_effect with each strips_pr_effect
  for (size_t i = 0; i < pr_size; i++) {
    strips_pr_effect[i].add |= strips_effect.add;
    strips_pr_effect[i].del |= strips_effect.del;
  }

  // construct cost vector
  std::vector<double> cost(metric_to_idx.size(), 0);
  for (auto const& name_val : ppddl_action.cost()) {
    auto const idx_it = metric_to_idx.find(name_val.first);
    assert(idx_it != metric_to_idx.end());
    assert(idx_it->second < cost.size());
    cost[idx_it->second] = name_val.second;
  }

  return {ppddl_action.name(), cost, strips_prec, strips_pr_effect, strips_pr_dist};
}

// Translate mdpsim problem into strips mossp problem
StripsMOSSP translateToStrips(MdpsimProblem const& problem) {
  using STRIPS::StripsState;

  // same as size() == 1 but with the methods provided. This is needed to make sure the AtomTable
  // only has atoms from a single problem.
  assert(++MdpsimProblem::begin() == MdpsimProblem::end());

  auto const& atom_table = Atom::getAtomTable();
  size_t n_atoms = atom_table.size();

  // The ordering is irrelevant but need to be fixed for the STRIPS representation this is the
  // ordering that will be used
  std::vector<Atom const*> ordered_atoms(n_atoms, nullptr);
  std::map<Atom const*, size_t> atom_to_idx;
  std::map<size_t, std::string> idx_to_name;

  for (size_t idx = 0; Atom const* const& atm : atom_table) {
    assert(atm != nullptr);
    ordered_atoms[idx] = atm;
    idx_to_name[idx] = atm->toString();
    atom_to_idx[atm] = idx;
    ++idx;
  }

  StripsState s0(n_atoms);
  for (auto init = problem.initialState(); Atom const* const& atm : init.atoms) {
    assert(atom_to_idx.find(atm) != atom_to_idx.end());
    s0.set(atom_to_idx[atm]);
  }

  StateFormula const& ppddl_goal = problem.goal();
  Mask goal_set = translateConjOfPosAtoms(ppddl_goal, atom_to_idx);

  std::map<std::string, size_t> metric_to_idx = buildMetricToIdxMap(problem);

  std::vector<StripsAction> actions;
  for (Action const* ppddl_action : problem.actions()) {
    // FWT: Somehow there are nullptr in problem.actions(). This is an issue from MDPSIM
    if (ppddl_action == nullptr) {
      continue;
    }
    actions.push_back(translateAction(*ppddl_action, atom_to_idx, metric_to_idx));
  }
  return {problem.name(), s0, goal_set, actions, idx_to_name};
}


std::map<std::string, size_t> buildMetricToIdxMap(Problem const& problem) {
  VecExpression const& metrics = problem.metrics();
  std::map<std::string, size_t> metric_to_idx;

#ifdef COST_ORDERING_AS_IN_PDDL_FUNCTION_STATEMENT
  // Making a vector of the function indexes so that we can sort it and be able to refer to each
  // index as its position in the ordered vector
  std::vector<int> function_index;
  for (Expression const* const& m : metrics) {
    Fluent const* fluent = dynamic_cast<Fluent const*>(m);
    if (!fluent) {
      std::cout << "Metric '" << m << "' is not a fluent and currently not supported\n";
      throw std::logic_error("Unsupported metric");
    }
    function_index.push_back(fluent->function().index());
  }
  std::sort(function_index.begin(), function_index.end());

  // The ordering of the metrics is the same as in the functions declaration, i.e., in the order
  // they appear in function_index. Keep in mind that one or more functions might not be in the
  // metrics
  for (Expression const* const& m : metrics) {
    Fluent const* fluent = dynamic_cast<Fluent const*>(m);
    assert(fluent);
    auto it = std::find(function_index.begin(), function_index.end(), fluent->function().index());
    assert(it != function_index.end());
    metric_to_idx[fluent->name()] = std::distance(function_index.begin(), it);
  }
#else
  // Ordering the cost vector according to the PDDL metric statement
  for (size_t idx = 0; Expression const* const& m : metrics) {
    // std::cerr << *m << "\n";
    Fluent const* fluent = dynamic_cast<Fluent const*>(m);
    if (!fluent) {
      std::cout << "Metric '" << m << "' is not a fluent and currently not supported\n";
      throw std::logic_error("Unsupported metric");
    }
    metric_to_idx[fluent->name()] = idx;
    ++idx;
  }
#endif

  return metric_to_idx;
}

}  // namespace PPDDL
