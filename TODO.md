TODO LIST
=========

### Gregor

#### MISCELLANEOUS
* remove hardware components from nodes into /hardware folder
* tf_updater and component put in /ros folder, also maybe rename them with ros_*
* update the grammar file in interpreter/grammar as it is in parser.py

#### 1. BUG/FEATURE (interpreter)
* string constant and other constants are not compatible because of sympy
* fix also print parser

* executor.py -> transform from point to point not tested
* remove the Type(Enum) class everywhere, its the same as type checking


#### 2. BUG/FEATURE (choreography)
* update choreography/grammar as in parser_chor.py
* move few checks from parser_chor (check is offline)
* collect all information (maybe create a graph) during parsing program
* return all the collected information in a function for check_chor (separate check from parsing)
* string problem with sypmy as in interpreter
* check_chor.py -> check_same_path_twice is checking every possible path twice if it visits the same loop again!
* separate CausalityTracker class in another file
* refactor ChoreographyCheck, rename class vars
* ast_proj.py -> fix the duration of motions, is shift_delay_check necessary?
* ast_chor.py remove Type(Enum)

#### 3. ROS BUGS/FEATURE
* when error in parser occurs stop executing programs
* make a script to configure all necessery things for running ros in python3?


### Damien

* frontend
  - parser keep track of positions (line/col) for error messages
* type system
  - process declaration: `name: type`
  - componenent signature: `name, var (id: type)*, mp ( id(arg: type [, arg: type]*) )*`
  - genration of ROS message signature from decalaration
* choreo
  - for the compat check, do a product construction directly on the global choreo
  - idle as variable duration mp
  - refactoring: spec/choreo folders need to be reorganized (same folder, better naming of the files, glue method to put thing together)
* runtime performance
  - have 1 thread for the program/ROS and 1 thread for the motion primitive
  - how long does sending/receive a message take in the current version?
  - more efficient interaction with tf (apply some code gen rather than sympy all the way, http://www.sympy.org/scipy-2017-codegen-tutorial/)
