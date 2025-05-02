#ifndef SSP_UTILS_H
#define SSP_UTILS_H


#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <cassert>

#include "../ext/mgpt/atom_states.h"  // to be able to define the hashes

class ConstrSSPIface;
class action_t;

/*
 * Useful typedefs
 */
#if defined(USE_PHMAP)
#include "../ext/parallel_hashmap/phmap.h"
using HashsetState = phmap::flat_hash_set<state_t>;
using HashsetActiontPtr = phmap::flat_hash_set<action_t const*>;
template<typename T>
using HashMapState = phmap::flat_hash_map<state_t, T>;
using HashActiontPtrToRational = phmap::flat_hash_map<action_t const*, Rational>;
#else
using HashsetState = std::unordered_set<state_t, hashState>;
using HashsetActiontPtr = std::unordered_set<action_t const*>;
template<typename T>
using HashMapState = std::unordered_map<state_t, T, hashState>;
using HashActiontPtrToRational = std::unordered_map<action_t const*, Rational>;
#endif

// Some useful Hash Maps from a state
using HashStateToRational = HashMapState<Rational>;
using HashStateToActiontPtrs = HashMapState<HashsetActiontPtr>;

using ListOfStates = std::list<state_t>;

// Hash Map from (s,a) to Rational: hashm[s][a] = Rational(x);
using HashStateActionToRational = HashMapState<HashActiontPtrToRational>;


/*
 * Returns an unordered set of state_t representing all the states reachable
 * from given states s in the given ssp.
 */
HashsetState const reachableStatesFrom(ConstrSSPIface const& ssp, state_t const& s);


/*
 * Return an applicable action selected at random.
 *
 * Guarantee: nullptr is only returned if there are no applicable action in s
 */
action_t const* randomAction(ConstrSSPIface const& ssp, state_t const& s);

#endif  // SSP_UTILS_H
