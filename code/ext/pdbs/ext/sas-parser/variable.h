#ifndef VARIABLE_H_PDB
#define VARIABLE_H_PDB

#include <iostream>
#include <vector>
using namespace std;

namespace FastDownwardParser_PDB {

class Variable {
    vector<string> values;
    string name;
    int layer;
    int level;
    bool necessary;
public:
    Variable(istream &in);
    int get_range() const;
    string get_name() const;
    int get_layer() const {return layer; }
    bool is_derived() const {return layer != -1; }
    void dump() const;
    string get_fact_name(int value) const {return values[value]; }
};

}
#endif
