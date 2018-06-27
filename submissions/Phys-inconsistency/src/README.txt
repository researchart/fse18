- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 ____  _   ___   ______  
|  _ \| | | \ \ / / ___| 
| |_) | |_| |\ V /\___ \ 
|  __/|  _  | | |  ___) |
|_|   |_| |_| |_| |____/ 
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

prop_phys_units.py  :  Main file that runs phys.
constraint_collector.py : collects various types of constraints with the help of tree_walker
constraint_scoper.py : scopes computed-unit constraints.
constraint_solver.py : translates collected constraints into factors.
cppcheckdata.py  :  Library to parse CPPCheck dump files, (parsed Code)
cps_constraints.py : data structures to store the collected constraints.
datamining.py : not used.
datamining2.py : collects naming constraints.
datamining_self_var2type.pkl : storage of priors (disabled usage)
datamining_self_vars.pkl : storage of priors (disabled usage)
error_checker.py   : from Phriky, traverses abstract syntax tree to find physical unit inconsistencies.
error_rechecker.py : from Phriky, traverses abstract syntax tree to find physical unit inconsistencies.
pgm/   : Probablistic graphical models from http://libDAI.org
str_utils.py  : helper functions for parsing strings
symbol_helper.py  : from Phriky, mapping between ROS attributes of shared libraries and Physical Unit Types (PUTs).
tree_walker.py : visitor pattern implementation to decorate the abstract syntax tree with PUTs.
unit_error.py : physical unit error container object.  One is generated per unit error.
unit_error_types.py : data structure to defind the different types of physical unit errors.
var_name_heuristic.py : (disabled usage)
