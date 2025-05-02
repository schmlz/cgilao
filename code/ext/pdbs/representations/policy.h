#ifndef MOLAO_STAR_POLICY_H_PDB
#define MOLAO_STAR_POLICY_H_PDB

#include <string>
#include <vector>
#include <set>
#include <map>

template <typename Problem>
class Policy {
 public:
  using State = typename Problem::State;
  using Action = typename Problem::Action;
  Policy() = default;

  bool contains(const State& state) const {
    return map_.contains(state);
  }

  void insert_action(const State& state, const Action& action) {
    if (!map_.contains(state)) {
      map_[state] = std::set<Action>();
    }
    map_[state].insert(action);
  }

  void set_actions(const State& state, const std::set<Action>& actions) {
    map_[state] = actions;
  }

  std::set<Action>& get_actions(const State& state) {
    return map_[state];
  }

  size_t size() const {
    return map_.size();
  }

  std::map<State, std::set<Action>> map() const {
    return map_;
  }

  std::map<State, std::set<Action>> map_;
};

#endif //MOLAO_STAR_POLICY_H
