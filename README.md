# PGCD - robot Programming and verification with Geometry, Concurrency, and Dynamics

__currently being ported to ROS 2 (not yet working)__

PGCD is a programming language and verifcation system for communication and execution of motion primitives within ROS nodes.

## Features

Currently the main functionalities of PGCD are:
* Communication between ROS nodes (ROS message & ROS topics)
* Execute motion primitive (CableRobot project by Marcus Pirron)
* Tracking frames (TF2 library)

## Project setup 

1.  Install ROS 2 Eloquent (tested with ubuntu 18.04)
2.  Install colcon: `sudo apt install python3-colcon-common-extensions`
3.  Install some extra python package: `pip install arpeggio numpy sympy`
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

## Running a Test

Let us run a simple example of two processes sending a message to each other.

To run the example you need to:

```
$ ros2 launch pgcd simple_example.launch.py
```

## PGCD Program structure

TODO:
- program
- motion primitive
- frame dependencies and shift
- launch file
- choreographic specification
- etc.

## Need to fix:

See [TODO.md](TODO.md)

## Verification

Aditionnally for the verification, the following are required:
* program: [dreal](https://github.com/dreal/dreal4), [spin](http://spinroot.com/spin/whatispin.html)
