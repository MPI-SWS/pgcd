# PGCD - robot Programming and verification with Geometry, Concurrency, and Dynamics

PGCD is a programming language and verification system for programming and verification of robotic choreographies.

TODO link to papers


## Features

TODO ...

## Status

Recently ported to ROS 2.
__Partially working.__

Currently there is some issue with the frame conversion.
(I cannot import `tf2_geometry_msgs` for some reason.)

The current workaround is to manually implement the appropriate transform and manually register it with `tf2_ros`.

## Project Setup

1.  Install ROS 2 Foxy (tested with ubuntu 20.04)
2.  Install colcon: `sudo apt install python3-colcon-common-extensions`
3.  Install some extra python package: `pip3 install arpeggio numpy sympy ply`
4.  Checkout this repository:
    ```bash
    cd
    git clone https://github.com/MPI-SWS/pgcd.git
    mkdir ~/ros2_ws
    ln -s ~/pgcd ~/ros2_ws/src
    ```
5.  Compile and source:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    . src/pgcd/install/local_setup.bash
    ```
    If you use PGCD often you can add the last operation to your `.bashrc`.
    ```bash
    echo "source ~/ros2_ws/src/pgcd/install/local_setup.bash" >> ~/.bashrc
    ```
6.  Running a Test.
    Let us run a simple example of two processes sending a message to each other.
    To run the example you need to:
    ```bash
    cd ~/ros2_ws
    export PYTHONPATH=$PYTHONPATH:.
    ros2 launch pgcd simple_example.launch.py
    ```

## PGCD Program structure

### Program

A PGCD program is written in an imperative style.
The control structure has branches,
```
if (expr) statement else statement
```
and loops.
```
while (expr) statement
```
Multiples statements can put one after the other with a block `{`, `}`.

The `else` branch of an if-else is mandatory.
If a branch is empty, just fill it with a `skip;` statement.

A program terminates after executing the last statement or when calling
```
exit(expr);
```

Currently, a program is monolithic and it is not possible to factor a program in multiple methods.

It is possible to store values in variables, i.e., `id = expr;`, where the expression is evaluated as a [sympy](https://www.sympy.org/) expression.
The variables have a global scope.
It is also possible to print expressions: `print(expr);` or `print(string);`.

Messages are sent with
```
send(dest, label, args);
```
and received with
```
receive(sender, motion) {
    case label(args) => statement
    case ...
}
```

Motion primitives are method calls: `id(args)`.
The parenthesis can be omitted when there is not arguments.

There are examples in the [experiments/sorting]() folder and details of the syntax can be found in the [parser](pgcd/nodes/interpreter/parser.py).

The semantics is mostly what you would expect for an imperative programming language.
The message passing layer and the motion primitives are the non-standard bits.

TODO explain ...

#### Physical Connections Between Robots

frame dependencies and shift

TODO this got broken in the port to ROS 2...

#### Motion Primitive

For the purpose of the runtime, the motion primitives are invoked as normal method within the python interpreter.
Therefore, they are currently implemented in python and put in the `PYTHONPATH`.

Uninterruptible motion primitive executes a single block until completion. 
Buy interruptible motion primitive should be implemented in a specific way.
We assume the interruptible motion primitive is a loop that read some sensor data, do some computation, and produces some outputs.
Currently, the primitive should do only one iteration of the loop.
The receive statement will executes it repeatedly until a message comes.

In the future, we will try to improve the API to execute interruptible motion primitives in a separate thread.

#### Running a Program

Executing a program with multiple robots is done with a ROS2 launch file.

The launch file itself is mostly boilerplate.
It has one node per robot which uses the `component.py` executable from the `pgcd` package.
As part of the launch is it also possible to define extra coordinate systems (`static_transform_publisher` from `tf2_ros`).

Then, there is a parameter files which tells PGCD what program to run for each robot (source file) and which file contains the implementation of the motion primitives.
The parameters also specifies the frame and mounting points for each robot.

In the  [experiments/sorting]() folder the launch file is [xp.launch.py](experiments/sorting/xp.launch.py) and the parameter file [params.yaml](experiments/sorting/params.yaml).

### Specification and Verification

For the specification and verification, it comes in two parts: a choreography for the communication protocols and which motion to execute.
Then, the motion primitives also need a specification.

#### Choreography

...

#### Model of the Robots

...

#### Motion Primitives Specification

...

Currently, we assume the motion primitive specifications but do not verify this.
There are plenty of work which focus on that and can be used there.
Our focus is how specify the communication and motions to help compositional verification.

## Need to fix:

See [TODO.md](TODO.md)

### Verification Dependencies

Additionally for the verification, the following are required:
* program: [dreal](https://github.com/dreal/dreal4), [spin](http://spinroot.com/spin/whatispin.html)
