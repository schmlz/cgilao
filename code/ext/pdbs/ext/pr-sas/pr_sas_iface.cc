#include <iostream>
#include <limits>
#include <vector>
#include <string>
#include <regex>

#include "pr_sas_iface.h"

#include "../mdpsim-parser/problems.h"
#include "../mdpsim-parser/pddl_to_strips.h"


namespace SasPlus {

SasPlusMOSSP::State translateState(
    state_t const& s,
    NonConditionalPrSasProblem const& hack_sas_problem,
    std::unordered_map<SasVariable, int> const& idxs,
    std::unordered_map<int, int> const& value_map)
{
  SasPlusState new_s(hack_sas_problem.variables().size());
  for (auto const& var : hack_sas_problem.variables()) {
    int const var_idx = idxs.at(var);
    int const val = value_map.at(var.valueAt(s));
    new_s[var_idx] = val;
  }
  return new_s;
}

SasPlusPartialState translate_hack_state_to_partial_state(
  SasValuation const& valuation, std::unordered_map<SasVariable, int> const& idxs, std::unordered_map<int, int> const& value_map)
{
  SasPlusPartialState part_state;
  for (auto const& [var, val] : valuation) {
    int const var_idx = idxs.at(var);
    part_state[var_idx] = value_map.at(val);
  }
  return part_state;
}

SasPlusMOSSP multiObjectiveSSPFromHackedPrSas(
  NonConditionalPrSasProblem const& hack_sas_problem,
  std::unordered_map<SasVariable, int> & idxs,
  std::unordered_map<int, int> & value_map)
{
  // -> idxs tracks the index of each variable
  // -> hack_value_to_new_value maps values onto the range 1,..., |domain|
  idxs.clear();
  value_map.clear();

  std::string name = hack_sas_problem.name() + "_adapted_to_MOSSP";

  std::vector<SasPlusVariable> variables;
  int idx = 0;
  for (auto const& hack_var : hack_sas_problem.variables()) {

    // Create new variable
    variables.push_back({
      idx,
      hack_var.name(),
      static_cast<VariableDomSize>(hack_var.domain().size())
    });
    idxs[hack_var] = idx;

    // Track how old values correspond to new values
    size_t val_idx = 0;
    for (auto const& val : hack_var.domain()) {
      value_map[val] = val_idx;
      ++val_idx;
    }

    ++idx;
  }

  SasPlusState s0(variables.size());
  for (auto const& [var, val] : hack_sas_problem.initial_state()) {
    int const var_idx = idxs[var];
    s0[var_idx] = value_map[val];
  }

  SasPlusPartialState goal = translate_hack_state_to_partial_state(hack_sas_problem.goal(), idxs, value_map);

  std::vector<SasPlusAction> actions;
  for (auto const& hack_action : hack_sas_problem.actions()) {
    SasPlusAction::Cost cost;
    for (auto const& hack_cost : hack_action.costVector()) {
      cost.push_back(hack_cost.double_value());
    }

    SasPlusPartialState prec = translate_hack_state_to_partial_state(hack_action.prec(), idxs, value_map);

    SasPlusAction::PrEffect pr_effects;
    SasPlusAction::PrDist pr_probs;
    for (size_t i = 0; i < hack_action.size(); ++i) {
      pr_effects.push_back(translate_hack_state_to_partial_state(hack_action.eff(i), idxs, value_map));
      pr_probs.push_back(hack_action.pr(i).double_value());
    }

    // ADD THE ACTION
    actions.emplace_back(SasPlusAction(hack_action.name(), cost, prec, pr_effects, pr_probs));
  }

  return SasPlusMOSSP(name, variables, s0, goal, actions);
}


SasPlusMOSSP multiObjectiveSSPFromPPDDL(std::string const& domain_fname,
                                        std::string const& problem_fname,
                                        std::string const& translate_py_path)
{
  using MdpsimProblem = PPDDL_PDB::Problem;

  /*
   * Parsing the original PPDDL_PDB problem
   */
  MdpsimProblem const* problem = PPDDL_PDB::parsePPDDL(domain_fname, problem_fname);
  assert(problem);

  /*
   * Generating the all-outcome translation PDDL file with some simplifications (i.e., no costs) to
   * pass to the fast downward parser
   */
  DetPDDL_PDB all_out_pddl_info = PPDDL_PDB::buildDeteterministicPDDLFiles(*problem,
                                        problem->initialState(),
                                        PPDDL_PDB::ALL_OUTCOMES,
                                        "/tmp/");

  /*
   * Run fast downward translate.py to get the plain text representation of the deterministic sas
   * problem and parse the text representation to a simple data structure
   */
  FastDwSasProblem_PDB det_sas_prob = fastDownwardTranslateAndParse(all_out_pddl_info,
                                                                translate_py_path);


  // The translated (det) SAS+ problem has axioms and they are NOT supported
  assert(det_sas_prob.axioms.size() == 0);

  /*
   * translate the simple representation of the deterministic SAS problem to our internal
   * probabilistic SAS representation
   */
#if 0  // Enable for displaying both all-out SAS+ and reconstructed MO-PR-SAS+
  SasPlusMOSSP translated(translateToPrSas(det_sas_prob, all_out_pddl_info));
  debug(det_sas_prob, translated);
  return translated;
#else
  return translateToPrSas(*problem, det_sas_prob, all_out_pddl_info);
#endif

}


FastDwSasProblem_PDB fastDownwardTranslateAndParse(DetPDDL_PDB const& det_pddl,
    std::string const& translate_bin)
{
  // FWT: using the modified version of translate.py that:
  //  = Has the -o flag and -o - redirect the problem to the stdout
  //  = Has the -q flag that turns off most of the output
  std::string cmd = translate_bin + " -q -o - "
                    + det_pddl.domain_file_path + " "
                    + det_pddl.problem_file_path;

  std::cout << "[" << __FUNCTION__ << "] executing: '" << cmd << "'" << std::endl;
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);

  if (!pipe) {
    std::cout << "[" << __FUNCTION__ <<"] Failed to open pipe to cmd = '" << cmd << "'\n"
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
    std::cout << "[" << __FUNCTION__ << "] Failed to find header.\n"
              << "Check if " << translate_bin << " exists. If not, use the option "
              << "'--translate-path' to change it.\nQuitting" << std::endl;
    exit(-1);
  }

  return FastDwSasProblem_PDB(sstream_cleaned_file);
}


void debug(FastDwSasProblem_PDB const& det_sas_prob, SasPlusMOSSP const& translated) {
  std::cout << "=================== det_sas_prob ======================\n";
  det_sas_prob.dump();
  std::cout << "=================== SasPlusMOSSP ======================\n";
  std::cout << "Variables\n";
  for (auto const& v : translated.variables()) {
    std::cout << "  " << v.name << " |D_v| =  " << v.domain << std::endl;
  }
  std::cout << "Initial state: " << translated.initialState() << std::endl;
  std::cout << "Goal Cond: " << translated.goal() << std::endl;
  std::cout << "Actions\n";
  for (auto const& a : translated.allActions()) {
    std::cout << a.name() << std::endl;
    std::cout << "  cost: ";
    for (double const& c_i : a.cost()) {
      std::cout << c_i << " ";
    }
    std::cout << std::endl;
    std::cout << "  prec: " << a.precondition() << std::endl;
    std::cout << "  effs:\n";
    auto const& pr = a.probabilities();
    auto const& eff = a.pr_effects();
    assert(pr.size() == eff.size());
    for (size_t i = 0; i < pr.size(); ++i) {
      std::cout << "    " << pr[i] << ": " << eff[i] << std::endl;
    }
  }
  std::cout << "=======================================================" << std::endl;
}


SasPlusMOSSP translateToPrSas(PPDDL_PDB::Problem const& original_problem,
    FastDwSasProblem_PDB const& det_sas_prob,
    DetPDDL_PDB const& det_pddl)
{
  std::cout << "[" << __FUNCTION__ << "] Starting translateToPrSas" << std::endl;

  std::string name{"TODO"};

  auto& fd_vec_varptrs = det_sas_prob.variables;

  /*
   * Translating variables
   */
  std::vector<SasPlusVariable> variables;
  // Helper map that associate each FD variable point to its index in fd_vec_varptrs, thus, its
  // index in variables
  std::unordered_map<FastDownwardParser_PDB::Variable*, size_t> var_ptr_to_idx;
  for (int idx = 0; auto const& var_ptr : fd_vec_varptrs) {
    assert(var_ptr);
    assert(var_ptr->get_range() < std::numeric_limits<VariableDomSize>::max());
    variables.push_back({idx, var_ptr->get_name(),
                         static_cast<VariableDomSize>(var_ptr->get_range())});
    var_ptr_to_idx[var_ptr] = idx++;
  }

  /*
   * Translating initial state
   */
  SasPlusState s0(variables.size());
  assert(s0.size() == variables.size());
  for (size_t i = 0; i < variables.size(); ++i) {
    int value = det_sas_prob.initial_state[fd_vec_varptrs[i]];
    assert(value < std::numeric_limits<VariableDomSize>::max());
    s0[i] = static_cast<VariableDomSize>(value);
  }


  /*
   * Translating goal
   */
  SasPlusPartialState goal;
  for (auto const& pair : det_sas_prob.goals) {
    auto const& fd_varptr = pair.first;
    int value = pair.second;
    assert(fd_varptr);
    assert(value < std::numeric_limits<VariableDomSize>::max());
    assert(var_ptr_to_idx.find(fd_varptr) != var_ptr_to_idx.end());
    size_t sas_var_idx = var_ptr_to_idx[fd_varptr];
    goal[sas_var_idx] = static_cast<VariableDomSize>(value);
  }

  /*
   * Translating the Fast Downward actions and rebuilding the all-outcome
   * determinization into the probabilistic representation.
   */
  std::vector<SasPlusAction> actions;
  rebuildActions(original_problem, det_sas_prob, det_pddl, variables, var_ptr_to_idx, actions);

  //  dump();

  return SasPlusMOSSP(name, variables, s0, goal, actions);
}


SasPlusAction::Cost mdpsimCostToSasPlusCost(
    PPDDL_PDB::CostMap const& cost_map,
    std::map<std::string, size_t> const& metric_to_idx)
{
  SasPlusAction::Cost cost(metric_to_idx.size(), 0);
  for (auto const& name_val : cost_map) {
    auto const idx_it = metric_to_idx.find(name_val.first);
    assert(idx_it != metric_to_idx.end());
    assert(idx_it->second < cost.size());
    cost[idx_it->second] = name_val.second;
  }
  return cost;
}


void rebuildActions(PPDDL_PDB::Problem const& original_problem,
                    FastDwSasProblem_PDB const& det_sas_prob,
                    DetPDDL_PDB const& det_pddl,
                    std::vector<SasPlusVariable> const& variables,
                    std::unordered_map<FastDownwardParser_PDB::Variable*, size_t> var_ptr_to_idx,
                    std::vector<SasPlusAction>& actions)
{
  struct PartialAction {
    SasPlusAction::Cost cost;
    SasPlusPartialState prec;
    SasPlusAction::PrEffect pr_effects;
    SasPlusAction::PrDist pr_probs;
  };

  // Building a hash of ground action names to Action const* (i.e., a grounded action)
  using MdpsimAction = PPDDL_PDB::Action;
  std::unordered_map<std::string, MdpsimAction const*> name_to_actionptr;
  for (MdpsimAction const* action : original_problem.actions()) {
    if (action == nullptr) continue;
    assert(name_to_actionptr.find(action->name()) == name_to_actionptr.end());
    name_to_actionptr[action->name()] = action;
  }
  // Translation map from function metric/function name to the expected vector index in the SAS+
  // representation (same as in the STRIPS representation)
  std::map<std::string, size_t> metric_to_idx = buildMetricToIdxMap(original_problem);

  std::map<std::string, PartialAction> partial_actions;
  std::regex has_prob_eff_number("-[0-9]+$", std::regex::egrep);

  // All the operators/action from the deterministic SAS+ representations are
  // effects for the probabilistic representation
  for (auto const& eff : det_sas_prob.operators) {
    // Full name of the effect, e.g., put-on-block-no-detonated-1 b5 b3
    std::string const eff_full_name(eff.get_name());
    // Effect name (i.e., without parameters), e.g., put-on-block-no-detonated-1
    std::string const eff_name = eff_full_name.substr(0, eff_full_name.find(" "));
    // Action parameters, e.g., b5 b3
    std::string const eff_params = eff_full_name.substr(eff_full_name.find(' ') + 1);
    // Action schema name (i.e., without the effect numbering), e.g., put-on-block-no-detonated
    std::string pddl_action_name = eff_name;
    if (std::regex_search(eff_name, has_prob_eff_number)) {
      pddl_action_name = eff_name.substr(0, eff_name.find_last_of('-'));
    }
    // Reconstructed grounded name (i.e., no effect index) , e.g., put-on-block-no-detonated b5 b3
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

    // bool new_pr_action = false;

    auto it = partial_actions.find(grounded_action_name);
    if (it == partial_actions.end())
    {
      // First time we see an effect of this action
      PartialAction pa{};
      // In SAS+ the precondition of an action is called prevail conditions. Here
      // we keep the more meaningful name (precondition).
      for (auto const& prevail : eff.get_prevail()) {
        assert(var_ptr_to_idx.find(prevail.var) != var_ptr_to_idx.end());
        size_t var_idx = var_ptr_to_idx[prevail.var];
        int value = prevail.prev;
        assert(value < variables[var_idx].domain);
        pa.prec[var_idx] = value;
      }
      auto it_action_ptr = name_to_actionptr.find(grounded_action_name);
      assert(it_action_ptr != name_to_actionptr.end());
      pa.cost = mdpsimCostToSasPlusCost(it_action_ptr->second->cost(), metric_to_idx);
      it = partial_actions.insert(it, {grounded_action_name, pa});
    }

    PartialAction& pa = it->second;

    // Translating each (potentially) conditional effect
    SasPlusPartialState det_eff;
    for (auto const& prepost : eff.get_pre_post()) {
      assert(var_ptr_to_idx.find(prepost.var) != var_ptr_to_idx.end());
      size_t var_idx = var_ptr_to_idx[prepost.var];
      //assert(pa.prec.find(var_idx) == pa.prec.end());

      // Merging the precondition of each atom. Notice that we will enforce that
      // they are the same over all the different probabilistic effects
      if (prepost.pre != -1) {
        int pre_value = prepost.pre;
        assert(pre_value < variables[var_idx].domain);
        auto it = pa.prec.find(var_idx);
        if (it == pa.prec.end()) {
          pa.prec.insert(it, {var_idx, pre_value});
        }
        else {
          assert(it->second == pre_value);
        }
      }

      // TODO(fwt): The current MO-PR-SAS+ representation does not allow conditional effect. Thus
      // blocking it with the assert. In order to support it, the code below needs to be adapted
      assert(!prepost.is_conditional_effect);
      /*
      SasValuation cond;
      if (prepost.is_conditional_effect) {
        for (auto const& ci : prepost.effect_conds) {
          SasVariable const& c_var = variables_[fd_varptr_to_var_idx.find(ci.var)->second];
          SasVarValue value = c_var.domain()[ci.cond];
          assert(cond.find(c_var) == cond.end());
          cond[c_var] = value;
        }
      }
      */

      assert(prepost.post >= 0);
      assert(det_eff.find(var_idx) == det_eff.end());
      assert(prepost.post < variables[var_idx].domain);
      det_eff[var_idx] = prepost.post;
    }  // for each simple or conditional effect

    auto it_det_eff = det_pddl.action_info.find(eff_name);
    assert(it_det_eff != det_pddl.action_info.end());
    pa.pr_effects.push_back(det_eff);
    pa.pr_probs.push_back(it_det_eff->second.prob);
  }

  // Creating the final actions and completing its probabilities with NOOPs
  for (auto& pair : partial_actions) {
    auto const& name = pair.first;
    PartialAction& pa = pair.second;
    // ASSUMPTION(fwt): Every action 'skipped' fast downward (i.e., not present in
    // output) are equivalent to NOOP (self-loop)
    //
    // Completing the actions with the NOOP effect when needed
    double noop_pr = 1;
    for (double p : pa.pr_probs) {
      noop_pr -= p;
    }
    if (noop_pr > 1e-5) {
      pa.pr_probs.push_back(noop_pr);
      pa.pr_effects.push_back({});
    }

    actions.emplace_back(SasPlusAction(name, pa.cost, pa.prec, pa.pr_effects, pa.pr_probs));
  }
}

}  // namespace SasPlus

