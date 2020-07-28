TODO LIST
=========

* ROS 2 port
  - ROS 2 Foxy Fitzroy (see changes in https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/) and Ubuntu 20.04
  - more test
  - take advantage of discovery (`export ROS_DOMAIN_ID=PGCD` ?)
* frontend
  - string constant and other constants are not compatible because of sympy
  - parser keep track of positions (line/col) for error messages
  - automatically generate frame shift for custom messages (currently limited to some predefined types from geometry package)
* type system
  - process declaration: `name: type`
  - componenent signature: `name, var (id: type)*, mp ( id(arg: type [, arg: type]*) )*`
  - genration of ROS message signature from decalaration
* choreo
  - thread partition
    * when traversing and find a fork -> get the paths to the next join
    * find all the processes in each branch and check it is a partition
    * traverse the branches and check all the processes in that branch have MP
  - for the compat check, do a product construction directly on the global choreo
  - variable duration mp (idle): generalize the causality for time intervals
* runtime performance
  - have 1 thread for the program/ROS and 1 thread for the motion primitive
  - more efficient interaction with tf (apply some code gen rather than sympy all the way, http://www.sympy.org/scipy-2017-codegen-tutorial/)
  - interpreter.py, `visit_receive` and `visit_motion`: better way to execute motions with messages (spawn a separate thread/process for the motion and interrupt it when done)

