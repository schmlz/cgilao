# Work in Progress

This code should be considered Work-in-Progress. There are no major issues that we are aware of, but we provide no guarantees. There are still some optimisations, refactors, and overall cleanup that we would like to do.

# Setup

You always need the "Core Setup." If you just want to play around with CG-iLAO* and the other algorithms with a restricted set of heuristics, then this is enough.

For the following independent features, you need the additional extra setup:
* The ROC heuristic requires "LP Solver"
* CG-iLAO* with FF expansions requires "FF"

This code is known to work on ubuntu 22.04.

## Core Setup

1. make sure your c++ compiler supports c++20 (GCC 10 and later should support all the features we need)
1. install the boost library (in ubuntu: `apt install libboost-dev`)
1. update the `build.py` file (`code/build.py`) -- look for the comment `### Host personalization` and following the existing examples there add an elif clause where you specify
	* `C_COMPILER` (find this out by typing `gcc --version` into terminal)
	* `CPP_COMPILER` (find this out by typing `g++ --version` into terminal)
	* `n_threads` (note: this is buggy for values > 1, so if you get linking errors just retry the build or set this to 1)
1. run ``python3 build.py --opt --ndebug ssp`` to build the relevant part of the code

---

Now you can run most algorithms with most heuristics (except ROC). For example:

* `./solver_ssp -p ilao -h lm-cut -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`
* `./solver_ssp -p lrtdp -h pdb2 -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`
* `./solver_ssp -p cg-ilao -h h-max -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`

## LP Solver (requirement for ROC heuristic)

Our code is compatible with gurobi and cplex. Note that we used cplex for our experiments.

### Cplex

Cplex is easier to set up.

1. download cplex from IBM's website and install it following their instructions. By default, this should produce an install directory `<cplex home>` that looks like `/opt/ibm/ILOG/CPLEX_Studio201/`
1. update the `build.py` file -- in the elif block you added in the core setup, now add
	* `cplex_directory = <cplex home>/cplex`
	* `cplex_concert_directory = <cplex home>/concert`
1. run `build.py` and append the flag `--lp_solver cplex`
	* e.g. `python3 build.py --opt --ndebug --lp_solver cplex ssp`

### Gurobi

Gurobi is a bit trickier, because you have to set up a license. You can look at their installation guide to see how to do it. If you have gurobi installed, you should have a home directory `<gurobi home>` that looks like `/opt/gurobi1103/linux64`. Then,


1. update the `build.py` file -- in the elif block you added in the core setup, now add
	* `cplex_directory = <cplex install>/cplex`
	* `cplex_concert_directory = <cplex install>/concert`
    * `gurobi_cpp_library = <gurobi home>/lib/libgurobi_g++X.Y.a`
    * `gurobi_home = <gurobi home>`
    * `gurobi_version = Z`
1. run `build.py` and append the flag `--lp_solver gurobi`
	* e.g. `python3 build.py --opt --ndebug --lp_solver gurobi ssp`

---

Now you can run algorithms with ROC. For example


* `./solver_ssp -p ilao -h roc -R 1 --max_rss_kb 4000000 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`
* `./solver_ssp -p lrtdp -h dead-end:roc -R 1 --max_rss_kb 4000000 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`

With cplex you need to specify `--max_rss_kb`.

Note: `dead-end:roc` is ROC with `h-max` as a dead-end detector.

## FF (requirement for FF expansions)

1. download and set up FF from [https://fai.cs.uni-saarland.de/hoffmann/ff.html](https://fai.cs.uni-saarland.de/hoffmann/ff.html), following their instructions. This should give you a binary file called `ff_mod`
1. place `ff_mod` into this directory (`code/`)

---

Now you can run CG-iLAO* with FF expansions. For example

* `./solver_ssp -p cg-ilao-extended:1:ff:postorder:1:-1:0 -h lm-cut -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`
* `./solver_ssp -p cg-ilao-extended:1:ff-mlo:postorder:1:-1:0 -h lm-cut -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`

# Running the code

You can run the code with `./solver_ssp -p <planner> -h <heuristic> -R 1 <domain> <problem>` where
* `<planner>` is the planner you want, e.g., `lrtdp`, `ilao`, `cg-ilao`
* `<heuristic>` is the heuristic you want, e.g., `h-max`, `lm-cut`, `pdb2`, `roc`, `dead-end:roc`
* `<domain>` is a filepath leading to the PDDL domain file
* `<problem>` is a filepath leading to the PDDL problem file

---

Some examples:

* `./solver_ssp -p ilao -h lm-cut -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`
* `./solver_ssp -p lrtdp -h pdb2 -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`
* `./solver_ssp -p cg-ilao -h h-max -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`

---

The extended versions of CG-iLAO* are specified `cg-ilao-extended:1:<expansion>:postorder:1:-1:<action elim>` where
* `<expansion>` is the expansion mechanism, e.g., `bellman` (this is single greedy expansion), `bellman-tied`, `complete` (aka all), `trial:100`, `ff`, `ff-mlo`
* `<action elim>` is a flag `0/1` that determines whether action elimination is enabled or not

The other parameters also do stuff, but that hasn't been discussed anywhere.

---

Some examples

* `./solver_ssp -p cg-ilao-extended:1:bellman-tied:postorder:1:-1:0 -h lm-cut -R 1 ../benchmarks/intermediate-triangle-tireworld/tire.domain.pddl ../benchmarks/intermediate-triangle-tireworld/tire3-start1.pddl`

# Some pointers

The code implementing CG-iLAO* and the other algorithms is in `planners`. Some interesting files:

* `cg-ilao.h/cc` -- this is the basic implementation of CG-iLAO*. It is the code that was used for the AAAI 2024 paper (after some cleanup and refactoring).
* `cg-ilao-extended.h/cc` -- this contains all the variants of CG-iLAO* that we considered in the journal paper.
* `ilao.h/cc` -- this is the implementation of iLAO*
* `lrtdp.h/cc` -- this is the implementation of LRTDP

A good place to start to understand these algorithms is the `solve()` method.

# Modifying Code

CAREFUL: the SAS+ parser in `ext/sas-parser` and `ext/pdbs/ext/sas-parser` is the same code but duplicated. If you modify one, make sure to modify the other accordingly. This is at the top of our list of refactoring to-dos.

# Note on Faster Hashmaps

In our experiments, we used a faster hashing function (komihash) and a more efficient version of hashmaps (phmap):

1. komihash from [https://github.com/avaneev/komihash](https://github.com/avaneev/komihash) (see `/ext/faster_hashing/`). This is distributed under the MIT license `/ext/faster_hashing/LICENSE`.
1. parallel hashmaps (phmap) from [https://github.com/greg7mdp/parallel-hashmap](https://github.com/greg7mdp/parallel-hashmap) (see `/ext/parallel_hashmaps/`). This is distributed under the Apache License 2.0 `/ext/parallel_hashmaps/LICENSE`.

These are included in the code, so you don't need to do anything extra.

# Acknowledgements

* komihash for faster hashing [https://github.com/avaneev/komihash](https://github.com/avaneev/komihash). It is distributed under the MIT license.
* phmap for faster hashtables [https://github.com/greg7mdp/parallel-hashmap](https://github.com/greg7mdp/parallel-hashmap). It is distributed under the Apache License 2.0.
* The PDB heuristic was adapted from [https://github.com/DillonZChen/cpp-mossp-planner](https://github.com/DillonZChen/cpp-mossp-planner), which itself adapts Thorsten Klößner's PDB code [https://zenodo.org/records/4604720](https://zenodo.org/records/4604720). That's distributed under FastDownward's GNU General Public license.
* The SAS+ parser and translator are from [FastDownward](https://www.fast-downward.org/latest/). They are distributed under FastDownward's GNU General Public License.
