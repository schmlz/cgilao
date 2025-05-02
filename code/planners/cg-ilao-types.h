#ifndef CGILAO_TYPES_H
#define CGILAO_TYPES_H

#include <vector>
#include <unordered_map>

#include "../ext/mgpt/actions.h"
#include "../ssps/bellman.h"
#include "../utils/die.h"
#include "../utils/mean_stdev_aggregator.h"
#include "../ext/mgpt/global.h"
#include "../ext/mgpt/hash.h"
#include "../ssps/ssp_iface.h"
#include "../ssps/ssp_utils.h"

#include "cg-dual-search-space.h"  // for the following:
// using SetStates = std::unordered_set<state_t, hashState>;
//
// struct StateActionPtr {
//   state_t state;
//   action_t const* action_ptr;
// };
//
// // Hash for StateAction. This is based on boost::hash_combine
// // http://www.boost.org/doc/libs/1_65_1/doc/html/hash/reference.html
// namespace std {
//   template<> struct hash<StateActionPtr> {
//     size_t operator()(StateActionPtr const& s_a) const {
//       size_t key = std::hash<state_t>()(s_a.state);
//       size_t action_ptr_key = std::hash<std::string>()(s_a.action_ptr ?
//                                                           s_a.action_ptr->name() : "");
//       key ^= action_ptr_key + 0x9e3779b9 + (key << 6) + (key >> 2);
//       return key;
//     }
//   };
//   template<> struct equal_to<StateActionPtr> {
//     bool operator()(StateActionPtr const& x, StateActionPtr const& y) const {
//       return (x.action_ptr == y.action_ptr) && (x.state == y.state);
//     }
//   };
// }
//

using VecActionPtr = std::vector<action_t const*>;

#if defined(USE_PHMAP)
#include "../ext/parallel_hashmap/phmap.h"
using SetOfConstrs = phmap::flat_hash_set<StateActionPtr>;
using SetOfStates = phmap::flat_hash_set<state_t>;
using StateToConstrs = phmap::flat_hash_map<state_t, SetOfConstrs>;
using MapStateToActionPtrs = phmap::flat_hash_map<state_t, VecActionPtr>;
using Policy = phmap::flat_hash_map<state_t, action_t const*>;
#else
using SetOfConstrs = std::unordered_set<StateActionPtr>;
using SetOfStates = std::unordered_set<state_t, hashState>;
using StateToConstrs = std::unordered_map<state_t, SetOfConstrs>;
// TODO(fwt): Improvment: v2:
// keep a bitset mask instead of a vector of pointer, then filter out
// in the applicable actions loop the actions outside the bitset
using MapStateToActionPtrs = std::unordered_map<state_t, VecActionPtr>;
using Policy = std::unordered_map<state_t, action_t const*>;
#endif


#endif // CGILAO_TYPES_H