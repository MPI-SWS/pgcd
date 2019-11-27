# PGCD - robot Programming and verification with Geometry, Concurrency, and Dynamics

PGCD is a programming language and verifcation system for programming and verification of robotic chroeographies.


## Features

TODO ...

## Status

Recently ported to ROS 2.
__Partially working.__

Currently there is some issue with the frame conversion.
(I cannot import `tf2_geometry_msgs` for some reason.)

The current workaround is to manually implement the appropriate transform and manually register it with `tf2_ros`.

## Project Setup 

1.  Install ROS 2 Eloquent (tested with ubuntu 18.04)
2.  Install colcon: `sudo apt install python3-colcon-common-extensions`
3.  Install some extra python package: `pip3 install arpeggio numpy sympy ply`
4.  Checkout this repository:
    ```
    $ cd
    $ git clone https://github.com/MPI-SWS/pgcd.git
    $ mkdir ~/ros2_ws
    $ ln -s ~/pgcd ~/ros2_ws/src
    ```
5.  Compile and source:
    ```
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
    $ . install/setup.bash
    ```
    If you use PGCD often you can add the last operation to your `.bashrc`.
    ```
    $ echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```
6.  Running a Test.
    Let us run a simple example of two processes sending a message to each other.
    To run the example you need to:
    ```
    $ cd ~/ros2_ws
    $ export PYTHONPATH=$PYTHONPATH:.
    $ ros2 launch pgcd simple_example.launch.py
    ```

## PGCD Program structure

TODO:
- program
- motion primitives (integration of the code, specification)
- frame dependencies and shift
- launch file
- choreographic specification
- etc.

## Need to fix:

See [TODO.md](TODO.md)

## Verification

Aditionnally for the verification, the following are required:
* program: [dreal](https://github.com/dreal/dreal4), [spin](http://spinroot.com/spin/whatispin.html)
