#ifndef STATE_H_PDB
#define STATE_H_PDB

#include <iostream>
#include <map>
#include <vector>
using namespace std;

namespace FastDownwardParser_PDB {

class Variable;

class State {
    map<Variable *, int> values;
public:
    State() {} // TODO: Entfernen (erfordert kleines Redesign)
    State(istream &in, const vector<Variable *> &variables);

    int operator[](Variable *var) const;
    void dump() const;
};
}

#endif
