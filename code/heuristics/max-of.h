#ifndef HEURISTICS_MAX_OF_H
#define HEURISTICS_MAX_OF_H

#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

#include "heuristic_iface.h"
#include "heuristic_cssp_iface.h"
#include "../ext/mgpt/states.h"

using VecSPtrHeuristic = std::vector<std::shared_ptr<heuristic_t>>;


class MaxOfHeuristics : public heuristic_t {
 public:
  MaxOfHeuristics(VecSPtrHeuristic heur_vec, bool prune_on_deadend)
    : heuristic_t("max-of"), heur_vec_(heur_vec), best_counter_(heur_vec.size(), 0),
      prune_on_deadend_(prune_on_deadend)
  {
    assert(heur_vec.size() > 0);
  }

  virtual ~MaxOfHeuristics() {
    for (size_t i = 0; i < heur_vec_.size(); ++i) {
      std::cout << "[" << name() << "] " << heur_vec_[i]->name() << " was best "
                << best_counter_[i] << " times" << std::endl;
    }
  }

 protected:
  double computeValue(state_t const& s) {
    double max = -1;
    bool tied = true;
    size_t best_idx = 0;
    for (size_t i = 0; i < heur_vec_.size(); ++i) {
      double v = heur_vec_[i]->value(s);
      if (prune_on_deadend_ && v == gpt::dead_end_value.double_value()) {
        break;
      }
      if (std::abs(v - max) <= 1e-4) {
        tied = true;
      }
      else if (v > max) {
        max = v;
        best_idx = i;
        tied = false;
      }
    }
    if (!tied) {
      best_counter_[best_idx]++;
    }
    return max;
  }

 private:
  VecSPtrHeuristic heur_vec_;
  std::vector<size_t> best_counter_;
  bool prune_on_deadend_;
};

using VecUPtrCSSPHeuristic = std::vector<HeuristicCSSPUniqPtr>;

class VecMaxOfHeuristic : public TimedHeuristicCSSPIface {
 public:
  VecMaxOfHeuristic(problem_t const& problem, VecUPtrCSSPHeuristic heur_vec,
      bool verbose)
    : TimedHeuristicCSSPIface(), heur_vec_(std::move(heur_vec)), verbose_(verbose), total_calls_(0)
  {
    for (size_t i = 0; i <  heur_vec_.size(); ++i) {
      best_counter_.push_back(std::vector<size_t>(problem.constraints().size() + 1, 0));
    }
  }

  virtual ~VecMaxOfHeuristic() {
    std::vector<size_t> ties(best_counter_[0].size(), total_calls_);
    for (size_t ih = 0; ih < heur_vec_.size(); ++ih) {
      std::cout << "[" << name() << "] " << heur_vec_[ih]->name() << " was best: ";
      for (size_t ic = 0; ic < best_counter_[ih].size(); ++ic) {
        std::cout << best_counter_[ih][ic] << " ";
        ties[ic] -=  best_counter_[ih][ic];
      }
      std::cout << std::endl;
    }
    std::cout << "[" << name() << "] " << " tied: ";
    for (size_t const& t : ties)
      std::cout << t << " ";
    std::cout << std::endl;
    statistics();
  }

  std::string name() const override {
    std::string name = "max-of";
    for (auto const& h : heur_vec_)
      name += ":" + h->name();
    return name;
  }


 protected:
  void computeValueVec(state_t const& s, std::vector<double>& h_val) override {
    total_calls_++;
    // Doing this to compute the max
    for (double& v : h_val) { v = -1.0; }

    size_t n = h_val.size();
    std::vector<double> aux(n, -1);
    std::vector<int> best_idx(n, -1);
    for (size_t i = 0; i < heur_vec_.size(); ++i) {
      heur_vec_[i]->valueVec(s, aux);
      if (verbose_) std::cout << "[max-of::" << heur_vec_[i]->name() << "]: [";

      for (size_t j = 0; j < n; ++j) {
        if (verbose_) std::cout << aux[j] << " ";
        if (std::abs(aux[j] - h_val[j]) <= 1e-4) {
          best_idx[j] = -1;
        }
        else if (aux[j] > h_val[j]) {
          h_val[j] = aux[j];
          best_idx[j] = i;
        }
      }
      if (verbose_) std::cout << "]" << std::endl;
    }

    assert(best_counter_[0].size() == n);
    for (size_t bi = 0; bi < n; ++bi) {
      int best = best_idx[bi];
      if (best == -1) continue;
      best_counter_[best][bi]++;
    }

    if (verbose_) {
      std::cout << "[max-of] h(" << s.toStringFull(gpt::problem) << ") = [";
      for (size_t i = 0; i < n; ++i) {
        int idx = best_idx[i];
        std::cout << h_val[i] << " ("
                  << (idx < 0 ? "tied" : heur_vec_[idx]->name())
                  << ") ";
      }
      std::cout << "]" << std::endl;
    }
  }



 private:
  VecUPtrCSSPHeuristic heur_vec_;
  bool verbose_;
  std::vector<std::vector<size_t>> best_counter_;
  size_t total_calls_;
};

#endif  // HEURISTICS_MAX_OF_H
