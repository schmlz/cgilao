#ifndef HEURISTICS_FROM_CSV_H
#define HEURISTICS_FROM_CSV_H

#include "heuristic_iface.h"

#include "../ext/mgpt/global.h"
#include "../ext/mgpt/rational.h"
#include "../ext/mgpt/states.h"
#include "../utils/die.h"
#include "../ssps/ssp_iface.h"

#include <iostream>


/*******************************************************************************
 *
 * constant value heuristic
 *
 ******************************************************************************/
class CSVHeuristic : public heuristic_t {
 public:
  CSVHeuristic(std::deque<std::string>& tokens)
    : heuristic_t("CSV-H")
  {
    // Set scale factor
    scale_factor_ = std::stof(tokens.front());
    tokens.pop_front();

    // Load CSV and place it into h_map_
    std::string csv_filepath = tokens.front();
    tokens.pop_front();
    std::ifstream file(csv_filepath);
    std::string line;
    if (file.is_open()) {
      while (std::getline(file, line)) {
        std::deque<std::string> s_h;
        boost::split(s_h, line, boost::is_any_of(","));
        if (s_h.empty()) { break; }
        const std::string s_str = s_h.front();
        s_h.pop_front();
        const double h = std::stod(s_h.front());
        s_h.pop_front();
        assert(s_h.empty());
        h_map_.emplace(s_str, h);
      }
      file.close();
    } else {
      std::cout << "ERROR: could not open heuristic csv file" << std::endl;
      exit(123);
    }
  }
  ~CSVHeuristic() { }

  /*
   * heuristic_t interface
   */
  double computeValue(state_t const& s) {
    const std::string s_str = s.toStringFull(gpt::problem);
    if (h_map_.find(s_str) == h_map_.end()) {
      std::cout << "ERROR: " << s_str << " not found in csv" << std::endl;
      exit(123);
    }
    return h_map_[s_str] * scale_factor_;
  }

 private:
  std::unordered_map<std::string, double> h_map_;
  double scale_factor_;
};

#endif // HEURISTICS_FROM_CSV_H
