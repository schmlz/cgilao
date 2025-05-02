#include <iostream>
#include <memory>
#include <cstdio>
#include <algorithm>
#include <string>
#include <regex>

#include "pr_sas_iface.h"

#include "../ext/mgpt/problems.h"
#include "../ext/mgpt/actions.h"

#include "../utils/utils.h"


int HackedPrSasProblem::total_none_of_those_ = 0;

HackedPrSasProblem::HackedPrSasProblem(problem_t const& problem, bool add_deadend_action)
    : mgpt_problem_(problem), name_("sas-fd-" + problem.name()), deadend_trans_applied_(false)
{
  /*
   * STEP 1: from PPDDL to all-outcome PDDL
   */
  DetPDDL det_pddl = buildDeteterministicPDDLFiles(ALL_OUTCOMES, &mgpt_problem_.s0());

  /*
   * STEP 2 and 3:
   *  - run fast downward translate.py to get the plain text representation of
   *    the deterministic sas problem
   *  - parse the text representation to a simple data structure
   */
  FastDwSasProblem det_sas_prob = fastDownwardTranslateAndParse(det_pddl,
                                                                gpt::translate_py_path);
  DIE(det_sas_prob.axioms.size() == 0,
      "The translated (det) SAS+ problem has axioms and they are NOT supported", -1);

  /*
   * STEP 4: translate the simple representation of the deterministic sas
   * problem to our internal probabilistic SAS representation
   */
  // Translate data-structure
  // - Get costs from problem
  // - Get probabilities from all_out_det_pddl_domain + 'prob_map'
  translateToPrSas(det_sas_prob, det_pddl);

  constrs_ = mgpt_problem_.constraints();
  if (add_deadend_action) {
    addDeadendAction();
  }
//  det_sas_prob.dump();
  // dump();
}


FastDwSasProblem HackedPrSasProblem::fastDownwardTranslateAndParse(
      DetPDDL const& det_pddl, std::string const& translate_bin) const
{
  // FWT: using the modified version of translate.py that:
  //  = Has the -o flag and -o - redirect the problem to the stdout
  //  = Has the -q flag that turns off most of the output
  std::string cmd = translate_bin + " -q -o - "
                    + det_pddl.domain_file_path + " "
                    + det_pddl.problem_file_path;

  std::cout << "[HackedPrSasProblem] executing: '" << cmd << "'" << std::endl;
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);

  if (!pipe) {
    std::cout << "[HackedPrSasProblem] Failed to open pipe to cmd = '" << cmd << "'\n"
              << "Check if translation.py is in the path shown above. "
              << "If not, use the option '--translate-path' to change it\n"
              << "Quitting" << std::endl;
    exit(-1);
  }

  std::stringstream sstream_cleaned_file;
  bool found_header = false;
  char* line = NULL;
  size_t len = 0;
  ssize_t read;
  while ((read = getline(&line, &len, pipe.get())) != -1) {
    if (!found_header && strncmp(line, "begin_version", 13) == 0) {
      found_header = true;
    }
    if (found_header) {
      sstream_cleaned_file << line;
    }
//    else {
//      std::cout << "Ignoring line '" << line;
//    }
  }
  if (line)
    free(line);

  if (!found_header) {
    std::cout << "[HackedPrSasProblem] Failed to find header.\n"
              << "Check if " << translate_bin << " exists. If not, use the option "
              << "'--translate-path' to change it.\nQuitting" << std::endl;
    exit(-1);
  }

  return FastDwSasProblem(sstream_cleaned_file);
}


MapFdValToAtomIdx HackedPrSasProblem::buildValToAtomTranslationMap() const {
  MapFdValToAtomIdx map;
  auto const& predicates = mgpt_problem_.domain().predicates();
  auto const& terms = mgpt_problem_.terms();
  for (atom_t ai = 0; ai < mgpt_problem_.number_atoms(); ++ai) {
    Atom const* atm = mgpt_problem_.atom_inv_hash_get(ai);
    std::string fd_name;
    if (mgpt_problem_.nprec() || ai % 2 == 0) {
      fd_name += "Atom ";
    }
    else {
      fd_name  += "NegatedAtom ";
    }
    fd_name += predicates.name(atm->predicate()) + "(";
    for (size_t ti = 0; ti < atm->arity(); ++ti) {
      if (ti > 0) {
        fd_name += ", ";
      }
      fd_name += terms.toString(atm->term(ti));
    }
    fd_name += ")";
//    std::cout << "idx = " << ai << " atom name = " << fd_name << std::endl;
    map[fd_name] = ai;
  }
  return map;
}


void HackedPrSasProblem::translateToPrSas(FastDwSasProblem const& det_sas_prob,
                                          DetPDDL const& det_pddl)
{
  std::cout << "[HackedPrSasProblem] Starting translateToPrSas" << std::endl;

  auto fd_varptr_to_var_idx = translateVariablesAndDomains(det_sas_prob);

  /*
   * Translating initial state
   */
  for (size_t i = 0; i < variables_.size(); ++i) {
    int val_idx = det_sas_prob.initial_state[det_sas_prob.variables[i]];
    SasVarValue const& val = variables_[i].domain()[val_idx];
    initial_state_[variables_[i]] = val;
  }


  /*
   * Translating goal
   */
  for (auto const& pair : det_sas_prob.goals) {
    SasVariable const& var = variables_[fd_varptr_to_var_idx[pair.first]];
    SasVarValue const& val = var.domain()[pair.second];
    goal_[var] = val;
  }

  // dump();

  /*
   * Translating the Fast Downward actions and rebuilding the all-outcome
   * determinization into the probabilistic representation.
   */
  rebuildActions(det_sas_prob, det_pddl, fd_varptr_to_var_idx);
}


MapFdVarPtrToIdx HackedPrSasProblem::translateVariablesAndDomains(
    FastDwSasProblem const& det_sas_prob)
{
  MapFdValToAtomIdx fd_val_to_atm = buildValToAtomTranslationMap();
  MapFdVarPtrToIdx fd_varptr_to_var_idx;

  for (auto const& var_ptr : det_sas_prob.variables) {
    SasVarDomain var_domain;
    bool has_none_of_those = false;
    SasVarValue none_of_those = 0;
    for (int i = 0; i < var_ptr->get_range(); ++i) {
      if (var_ptr->get_fact_name(i) == "<none of those>") {
        /*
         * <none of those> means that and of the negation of all the values in
         * this var happens together, i.e., either one of them is true or none.
         *
         */
        // none of those must be the last value and not the only one
        assert(i == var_ptr->get_range() - 1);
        assert(i > 0);
        has_none_of_those = true;
        none_of_those = --total_none_of_those_;
        var_domain.push_back(none_of_those);
        std::cout << "[HackedPrSasProblem] Found 'NONE OF THOSE' value for variable "
                  << var_ptr->get_name() << std::endl;
      }
      else {
        assert(fd_val_to_atm.find(var_ptr->get_fact_name(i)) != fd_val_to_atm.end());
        assert(fd_val_to_atm[var_ptr->get_fact_name(i)] >= 0);
        var_domain.push_back(fd_val_to_atm[var_ptr->get_fact_name(i)]);
      }
    }
    fd_varptr_to_var_idx[var_ptr] = variables_.size();
    if (has_none_of_those)
      variables_.emplace_back(var_ptr->get_name(), var_domain, none_of_those);
    else
      variables_.emplace_back(var_ptr->get_name(), var_domain);
    variables_.back().set_id(variables_.size()-1);
#ifndef NDEBUG
    // Make sure that var.id matches its position in variables_
    for (size_t i = 0; i < variables_.size(); ++i) {
      auto const& var = variables_[i];
      assert(var.id() == i);
    }
#endif
  }
  return fd_varptr_to_var_idx;
}


MapFdActionNameToActionTPtr HackedPrSasProblem::buildNameToActionTranslationMap() const
{
  MapFdActionNameToActionTPtr map;
  for (const auto& ai : mgpt_problem_.actionsT()) {
    // For grounded actions, the name already has the parameters
    std::string const& full_name = ai->name();
    assert(full_name[0] == '(');
    assert(full_name[full_name.length()-1] == ')');
//    std::cout << "full name = '" << full_name.substr(1, full_name.length() - 2) << "'\n";
    map[full_name.substr(1, full_name.length() - 2)] = ai;
  }
  return map;
}


void HackedPrSasProblem::rebuildActions(FastDwSasProblem const& det_sas_prob,
    DetPDDL const& det_pddl, MapFdVarPtrToIdx const& fd_varptr_to_var_idx)
{
  /*
   * Translating Actions
   */
  // ASSUMPTION(fwt): mgpt_problem_ must contain a flatted and grounded problem.
  DIE(mgpt_problem_.actionsT().size() > 0, "mgpt problem has no grounded actions", -1);

  std::unordered_map<std::string, size_t> name_to_pr_action_idx;

  auto action_name_to_ptr = buildNameToActionTranslationMap();
  std::regex has_prob_eff_number("-[0-9]+$", std::regex::egrep);

  // All the operators/action from the deterministic SAS+ representations are
  // effects for the probabilistic representation
  for (auto const& eff : det_sas_prob.operators) {
    std::string const eff_full_name(eff.get_name());
    std::string const eff_name = eff_full_name.substr(0, eff_full_name.find(" "));
    std::string const eff_params = eff_full_name.substr(eff_full_name.find(' ') + 1);
    std::string pddl_action_name = eff_name;
    if (std::regex_search(eff_name, has_prob_eff_number)) {
      pddl_action_name = eff_name.substr(0, eff_name.find_last_of('-'));
    }
    std::string grounded_action_name = pddl_action_name;
    if (eff_params != "") {
      grounded_action_name += ' ' + eff_params;
    }

//    std::cout
//        << "eff_full_name = '" << eff_full_name << "'\n"
//        << "eff_name = '" << eff_name << "'\n"
//        << "pddl_action_name = '" << pddl_action_name << "'\n"
//        << "grounded_action_name = '" << grounded_action_name << "'\n"
//        << std::endl;

    bool new_pr_action = false;

    if (name_to_pr_action_idx.find(grounded_action_name) == name_to_pr_action_idx.end())
    {
      // First time we see an effect of this action
      new_pr_action = true;
      size_t idx = actions_.size();
      name_to_pr_action_idx[grounded_action_name] = idx;
      actions_.push_back(PrSasAction());
      PrSasAction& a = actions_[idx];
      a.name() = grounded_action_name;
      // The cost file and can be populated now since it is the same across all
      // effects (by ASSUMPTION)
      assert(action_name_to_ptr.find(grounded_action_name) != action_name_to_ptr.end());
      a.costVector() = action_name_to_ptr[grounded_action_name]->costVector();
      a.setOriginalAction(action_name_to_ptr[grounded_action_name]);
    }


    // In SAS+ the precondition of an action is called prevail conditions. Here
    // we keep the more meaningful name (precondition).
    SasValuation translated_prec;
    for (auto const& prevail : eff.get_prevail()) {
      SasVariable const& var = variables_[fd_varptr_to_var_idx.find(prevail.var)->second];
      SasVarValue value = var.domain()[prevail.prev];
      translated_prec[var] = value;
    }

    // Translating each (potentially) conditional effect
    VectorSasEffect det_eff;
    for (auto const& prepost : eff.get_pre_post()) {
      SasVariable const& var = variables_[fd_varptr_to_var_idx.find(prepost.var)->second];
      assert(translated_prec.find(var) == translated_prec.end());

//      std::cout << "prepost.pre == " << prepost.pre << std::endl;
      // Merging the precondition of each atom. Notice that we will enforce that
      // they are the same over all the different probabilistic effects
      if (prepost.pre != -1) {
        SasVarValue value = var.domain()[prepost.pre];
        assert(translated_prec.find(var) == translated_prec.end()
                || translated_prec[var] == value);
        translated_prec[var] = value;
      }

      SasValuation cond;
      if (prepost.is_conditional_effect) {
        for (auto const& ci : prepost.effect_conds) {
          SasVariable const& c_var = variables_[fd_varptr_to_var_idx.find(ci.var)->second];
          SasVarValue value = c_var.domain()[ci.cond];
          assert(cond.find(c_var) == cond.end());
          cond[c_var] = value;
        }
      }

      assert(prepost.post >= 0);
      VarAndValue simple_eff = {var, var.domain()[prepost.post]};
      det_eff.emplace_back(cond, simple_eff);
//      std::cout << det_eff[det_eff.size() - 1] << std::endl;
    }  // for each simple or conditional effect

    PrSasAction& a = actions_[name_to_pr_action_idx[grounded_action_name]];
    if (new_pr_action) {
      a.prec() = translated_prec;
    }
    else {
      assert(translated_prec == a.prec());
    }


#ifndef NDEBUG
    // DEBUG: making sure that the translated preconditions do not conflict with
    // the preconditions already in the action
    for (auto const& pair : translated_prec) {
      assert(a.prec().find(pair.first) == a.prec().end()
              || a.prec()[pair.first] == pair.second);
    }
#endif

    auto it_det_eff = det_pddl.action_info.find(eff_name);
    assert(it_det_eff != det_pddl.action_info.end());
    a.pushEffect(it_det_eff->second.prob, det_eff);
  }

  // ASSUMPTION(fwt): Every action 'skipped' fast downward (i.e., not present in
  // output) are equivalent to NOOP (self-loop)
  //
  // Completing the actions with the NOOP effect when needed
  for (auto& a : actions_) {
    Rational noop_pr = 1;
    for (size_t i = 0; i < a.size(); ++i) {
      noop_pr -= a.pr(i);
    }
    if (noop_pr > Rational(1, 100000)) {
      a.pushEffect(noop_pr, SasValuation());
    }
  }
}

void changeVarInState(SasVariable const& var, SasVarValue const& new_value, state_t& s,
                      problem_t const* mgpt_problem)
{
  assert(var.checkMutualExclusivityAt(s));

  SasVarValue cur_value = var.valueAt(s);

  // Checking if nothing will change for var
  if (cur_value == new_value) return;

  if (mgpt_problem == nullptr) {
    assert(gpt::problem != nullptr);
    mgpt_problem = gpt::problem;
  }

  /*
   * We need to change the state. This is simply making the old value false
   * and the new one true. Notice that, for mGPT, sometimes the odd atoms
   * represent the negation of the respective even atom ONLY when talking
   * about preconditions and NEVER in the state.
   */
  if (var.hasNoneOfThoseValue()
      && (cur_value == var.noneOfThoseValue() || new_value == var.noneOfThoseValue()))
  {
    // We need to deal with "none of those" value
    if (cur_value == var.noneOfThoseValue()) {
      // cur_value == none-of-those and new_value == K
      //   -> Turn K on because K was off
      if (mgpt_problem->nprec() || new_value % 2 == 0) {
        s.add(new_value);
      }
      else {
        // new_value is a negative atom and adding it is equivalent to remove
        // the positive version of it
        atom_t pos_atm = new_value - 1;
        assert(pos_atm >= 0);
        assert(s.holds(pos_atm));
        s.clear(pos_atm);
      }
    }
    else {
      // cur_value == K and new_value == none-of-those
      //   -> Turn K off because the mutually exclusivity means that
      //   none-of-those except K is true, thus removing K we get none-of-those
      assert(new_value == var.noneOfThoseValue());
      if (mgpt_problem->nprec() || cur_value % 2 == 0) {
        s.clear(cur_value);
        // Odd atoms should never be true in states
        assert(mgpt_problem->nprec() || !s.holds(cur_value+1));
      }
      else {
        // cur_value is a negative atom and clearing it is equivalent to add
        // the positive version of it
        // Odd atoms should never be true in states
        assert(!s.holds(cur_value));
        atom_t pos_atm = new_value - 1;
        assert(pos_atm >= 0);
        assert(!s.holds(pos_atm));
        s.add(pos_atm);
      }
    }
  }
  else {
    // Easy case: we don't have to deal with "none of those" value
    if (mgpt_problem->nprec()) {
      // No representation of negative atoms
      s.clear(cur_value);
      s.add(new_value);
    }
    else {
      // Odd atoms are the negation of even ones but they are NOT used in the
      // states
      s.clear(cur_value);
      if (new_value % 2 == 0) {
        s.add(new_value);
      }
    }
  }
  assert(var.checkMutualExclusivityAt(s));
}



state_t HackedPrSasProblem::applyVecSasEffTo(VectorSasEffect const& vec_eff,
    state_t const& s) const
{
  state_t s_prime(s);
  for (SasEffect const& c_eff : vec_eff) {
    if (!c_eff.isApplicable(s)) { continue; }
    changeVarInState(c_eff.eff().var, c_eff.eff().value, s_prime, &mgpt_problem_);
  }  // for each (conditional) effect
  return s_prime;
}


void HackedPrSasProblem::expand(PrSasAction const& a, state_t const& s,
    ProbDistStateIface& pr) const
{
  pr.clear();
  assert(a.isApplicable(s));
//  std::cout << "\nSAS expand(" << a.name() << ", " << s.toStringFull(&mgpt_problem_)
//            << "):\n";
  for (size_t ie = 0; ie < a.size(); ++ie) {
    VectorSasEffect const& eff = a.eff(ie);
    Rational const& p = a.pr(ie);
    pr.insert(applyVecSasEffTo(eff, s), p.double_value());
  }
#ifndef NDEBUG
  double total_p = 0;
  for (auto const& ip : pr) {
//    std::cout << "  " << ip.prob() << "    " << ip.event().toStringFull(&mgpt_problem_)
//              << std::endl;
    total_p += ip.prob();
  }
  assert(total_p > 0.99999);
  assert(total_p < 1.00001);
#endif
}


std::string sasVarValueToString(SasVarValue const& value) {
  /*
   * ASSUMPTION: Any value less than 0 represents "none of those"
   */
  if (value < 0) {
    return "(none-of-those)";
  }
  std::string rv;
  if (value % 2 == 1) {
    rv += "(not ";
  }
  rv += gpt::problem->atom_inv_hash_get(value)->toString();
  if (value % 2 == 1) {
    rv += ")";
  }
  return rv;
}


std::ostream& operator<<(std::ostream& os, SasValuation const& valuation) {
  for (auto const& it : valuation) {
    os << "(" << it.first.name() << " = " << it.second << " -- "
       << sasVarValueToString(it.second) << ") ";
  }
  return os;
}


std::ostream& operator<<(std::ostream& os, SasEffect const& eff) {
  os << "sas-eff: ";
  os << eff.eff().var.name() << " <- " << eff.eff().value << " -- "
     << sasVarValueToString(eff.eff().value);
  if (eff.isConditional()) {
    os << " | COND[" << eff.cond().size() << "]: " << eff.cond();
  }
  return os;
}


void HackedPrSasProblem::dump() const {
  std::cout << "name: " << name_ << "\n"
            << "Variables (" << variables_.size() << "):" << std::endl;

  size_t idx = 0;
  for (auto const& var : variables_) {
    std::cout << " " << (idx++) << ". " << var.name()
              << " -- domain (" << var.domain().size() << ")\n";
    size_t idx_domain = 0;
    for (auto const& d : var.domain()) {
      std::cout << "   " << (idx_domain++) << ". " << sasVarValueToString(d)
                << " = " << d << std::endl;
    }
  }

  std::cout << "Initial state (" << initial_state_.size() << "): "
            << initial_state_ << std::endl;

  std::cout << "Goal state (" << goal_.size() << "): "
            << goal_ << std::endl;

  idx = 0;
  std::cout << "Actions (" << actions_.size() << "):\n";
  for (auto const& a : actions_) {
    std::cout << " " << (idx++) << ". " << a.name() << std::endl
              << "   cost (" << a.costVector().size() << "):";
    for (auto const& c : a.costVector()) {
      std::cout << " " << c;
    }
    std::cout << std::endl
              << "   prec (" << a.prec().size() << "): "  << a.prec() << "\n"
              << "   effs (" << a.size() << "):\n";
    for (size_t pit = 0; pit < a.size(); ++pit) {
      std::cout << "     " << pit << ". Pr = " << a.pr(pit) << "\n";
      VectorSasEffect const& eff = a.eff(pit);
      for (size_t eit = 0; eit < eff.size(); ++eit) {
        std::cout << "       " << eit << ". " << eff[eit] << "\n";
      }
    }
  }
}

bool SasVariable::checkMutualExclusivityAt(state_t const& s) const {
  int total_matches = 0;
  for (SasVarValue const& v : domain()) {
    if ((!has_none_of_those_ || v != none_of_those_) && holdsNegationSafe(s, v)) {
      total_matches++;
    }
  }
  if (has_none_of_those_ && total_matches == 0) {
    total_matches = 1;
  }

  if (total_matches != 1) {
    std::cout << "Sas variable mutual exclusivity was broken! Variable '"
              << name() << "' has " << total_matches << " values true in "
              << "the given state s.\n"
              << "  Var Domain = {";
    for (auto const& d : domain()) {
      std::cout << " " << sasVarValueToString(d);
    }
    std::cout << "}\n  State s = " << s.toStringFull(gpt::problem)
              << "\nQuitting\n";
    return false;
  }
  return true;
}


bool SasVariable::hasRepeatedValue() const {
  std::set<SasVarValue> unique_values(domain_.begin(), domain_.end());
  return domain_.size() != unique_values.size();
}


NonConditionalPrSasProblem::NonConditionalPrSasProblem(
    HackedPrSasProblem const& sas_problem, bool add_deadend_action)
  : sas_problem_(sas_problem), deadend_trans_applied_(false)
{
  std::cout << "[NonConditionalPrSasProblem] Compiling conditionals away...\n";
  // Removing the conditionals
  for (PrSasAction const& a : sas_problem_.actions()) {
    compileAwayConditionalEffs(a);
  }
  std::cout << "[NonConditionalPrSasProblem] Done! Total actions: "
            << actions().size() << " ("
            << (actions().size() - sas_problem.actions().size())
            << " added)\n";

  if (add_deadend_action) {
    addDeadendAction();
  }
}

std::deque<SasValuation> powerSetValuation(
                       std::unordered_set<SasVariable>::const_iterator it,
                       std::unordered_set<SasVariable>::const_iterator const& end)
{
  std::deque<SasValuation> rv;
  if (it == end) {
    // TODO: performance
    SasValuation aux;
    rv.push_back(aux);
    return rv;
  }

  SasVariable const& var = *it;
  auto previous = powerSetValuation(++it, end);
  for (SasValuation& valuation : previous) {
    assert(valuation.find(var) == valuation.end());
    for (SasVarValue const& v : var.domain()) {
      valuation[var] = v;
      rv.push_back(valuation);
    }
  }
  return rv;
}


void NonConditionalPrSasProblem::compileAwayConditionalEffs(PrSasAction const& a) {
  std::unordered_set<SasVariable> conditioners = conditionersOfAction(a);
  std::deque<SasValuation> power_set_conditions = powerSetValuation(conditioners.begin(),
                                                                    conditioners.end());
  // The powerset of the empty set has size 1
  if (power_set_conditions.size() == 1) {
    actions_.emplace_back(a);
  }
  else {
    size_t i = 0;
    for (SasValuation const& cond : power_set_conditions) {
      if (i % 1000 == 0) {
        if (get_max_resident_mem_in_kb() > gpt::max_rss_kb) {
          EXIT("compileAwayConditionEffs used too much memory");
        }
      }
      actions_.emplace_back(a, cond, "-" + std::to_string(i++));
    }
  }
}

void NonConditionalPrSasProblem::expand(NonConditionalPrSasAction const& a, state_t const& s,
                                        ProbDistStateIface& pr) const
{
  pr.clear();
  assert(a.isApplicable(s));
//  std::cout << "\nSAS expand(" << a.name() << ", " << s.toStringFull(&mgpt_problem_)
//            << "):\n";
  for (size_t ie = 0; ie < a.size(); ++ie) {
    SasValuation const& eff = a.eff(ie);
    Rational const& p = a.pr(ie);
    pr.insert(applyValuationTo(eff, s), p.double_value());
  }
#ifndef NDEBUG
  double total_p = 0;
  for (auto const& ip : pr) {
//    std::cout << "  " << ip.prob() << "    " << ip.event().toStringFull(&mgpt_problem_)
//              << std::endl;
    total_p += ip.prob();
  }
  assert(total_p > 0.99999);
  assert(total_p < 1.00001);
#endif
}


state_t NonConditionalPrSasProblem::applyValuationTo(SasValuation const& valuation,
    state_t const& s) const
{
  state_t s_prime(s);
  for (auto const& pair : valuation) {
    changeVarInState(pair.first, pair.second, s_prime);
  }
  return s_prime;
}


void NonConditionalPrSasProblem::dump() const {
  std::cout << "name: " << name() << "\n"
            << "Variables (" << variables().size() << "):" << std::endl;

  size_t idx = 0;
  for (auto const& var : variables()) {
    std::cout << " " << (idx++) << ". " << var.name()
              << " -- domain (" << var.domain().size() << ")\n";
    size_t idx_domain = 0;
    for (auto const& d : var.domain()) {
      std::cout << "   " << (idx_domain++) << ". " << sasVarValueToString(d)
                << std::endl;
    }
  }

  std::cout << "Initial state (" << initial_state().size() << "): "
            << initial_state() << std::endl;

  std::cout << "Goal state (" << goal().size() << "): "
            << goal() << std::endl;

  idx = 0;
  std::cout << "Actions (" << actions_.size() << "):\n";
  for (auto const& a : actions_) {
    std::cout << " " << (idx++) << ". " << a.name() << std::endl
              << "   cost (" << a.costVector().size() << "):";
    for (auto const& c : a.costVector()) {
      std::cout << " " << c;
    }
    std::cout << std::endl
              << "   prec (" << a.prec().size() << "): "  << a.prec() << "\n"
              << "   effs (" << a.size() << "):\n";
    for (size_t pit = 0; pit < a.size(); ++pit) {
      std::cout << "     " << pit << ". Pr = " << a.pr(pit) << " " << a.eff(pit) << "\n";
    }
  }
}

