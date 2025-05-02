#ifndef MUTEX_GROUP_H_PDB
#define MUTEX_GROUP_H_PDB

#include <iostream>
#include <vector>
using namespace std;

namespace FastDownwardParser_PDB {

class Variable;

class MutexGroup {
    vector<pair<const Variable *, int>> facts;
public:
    MutexGroup(istream &in, const vector<Variable *> &variables);

    void dump() const;
};

}

#endif
