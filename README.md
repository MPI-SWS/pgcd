# PGCD - robot Programming and verification with Geometry, Concurrency, and Dynamics

PGCD is a programming language and verification system for programming and verification of robotic choreographies.

The goal of this project is to develop a kind of choreographic specification which help verify CPS by decomposing the verification task around the communication and using assume-guarantee contracts.

The main ideas can be found in the following papers:
[ICCPS 2019](https://dzufferey.github.io/files/2019_Motion_Session_Types_for_Robotic_Interactions_updated.pdf),
[ECOOP 2019](https://dzufferey.github.io/files/2019_PGCD_Robot_Programming_and_Verification_with_Geometry_Concurrency_and_Dynamics.pdf), and
[OOPSLA 2020](https://dzufferey.github.io/files/2020_Multiparty_Motion_Coordination_From_Choreographies_to_Robotics_Programs.pdf).

### Status

Runtime recently ported to ROS 2.
__Partially working.__

Currently there is some issue with the frame conversion.
(I cannot import `tf2_geometry_msgs` for some reason.)
This does not interfere with the examples described above.

The current workaround is to manually implement the appropriate transform and manually register it with `tf2_ros`.


### Project Setup

#### Runtime

1.  Install Python > 3.5 (tested with 3.8) and `pip`: `sudo apt install python3 python3-pip`
2.  [Install ROS 2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/) (tested with Ubuntu 20.04)
3.  Install colcon: `sudo apt install python3-colcon-common-extensions`
4.  Install some extra python package: `pip3 install arpeggio numpy sympy ply`
5.  Checkout this repository:
    ```bash
    cd
    git clone https://github.com/MPI-SWS/pgcd.git
    mkdir ~/ros2_ws
    ln -s ~/pgcd ~/ros2_ws/src
    ```
6.  Compile and source:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    . install/local_setup.bash
    ```
    If you use PGCD often you can add the last operation to your `.bashrc`.
    ```bash
    echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
    ```
7.  Running a Test.
    Let us run a simple example of two processes sending a message to each other.
    To run the example you need to:
    ```bash
    cd ~/ros2_ws
    export PYTHONPATH=$PYTHONPATH:`pwd`:.
    ros2 launch pgcd simple_example.launch.py
    ```

#### Verification Only (no Runtime)

1. Install Python > 3.5 (tested with 3.8) and `pip`: `sudo apt install python3 python3-pip`
2. Install some extra Python packages: `pip3 install arpeggio numpy sympy ply`
3. Install [dReal 4](https://github.com/dreal/dreal4) and make sure the `dreal` executable is in the path.
4. Checkout this repository:
    ```bash
    cd
    git clone https://github.com/MPI-SWS/pgcd.git
    ```
5.  Running a Test.
    ```bash
    cd pgcd/pgcd/nodes/verification
    ./run_tests.sh test/xp_fetch_02_test.py
    ```

Additionally for some older tests [spin](http://spinroot.com/spin/whatispin.html) is used (optional, not needed when using choreographic specifications).


### PGCD Program structure

#### Program

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

There are examples in the [experiments/sorting](experiments/sorting) folder and details of the syntax can be found in the [parser](pgcd/nodes/interpreter/parser.py).

The semantics is mostly what you would expect for an imperative programming language.
The message passing layer and the motion primitives are the non-standard bits.

The communication model is synchronous.
In practice, this means every message is followed by an `ack` and the sender blocks until it receives the acknowledgment.
We use that semantics as choreographies with asynchronous communication and time are not well-understood.
Right now, checking the choreographic specification includes information about the duration of the motion primitives to make sure that a receiver is ready to receive when a sender sends.

#### Physical Connections Between Robots

One design goal of PGCD is facilitating modular verification by making sure each robot's controller work in the components local frame.
The message are automatically updated to account for the frame shift between the sender and receiver.

__Unfortunately, this part got broken in the port to ROS 2. I'll fix it as soon as I have time.__
The messages are still send and received but the data they contain is not updated...

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

In the [experiments/sorting](experiments/sorting) folder the launch file is [xp.launch.py](experiments/sorting/xp.launch.py) and the parameter file [params.yaml](experiments/sorting/params.yaml).

### Specification and Verification

For the specification and verification, it comes in two parts: a choreography for the communication protocols and which motion to execute.
Then, the motion primitives also need a specification.

#### Choreography

Choreographies are expressed in a state machine-like form.
```
ChoreographyName =
def Transitions
in [ predicate ] InitialState
```

The transitions are of the following form:

* Message:
  ```
  state = sender -> receiver: message; state
  ```
* Motion primitive:
  ```
  state = (component: motion, ...); state
  ```
* Branch and merge:
  ```
  state = state + state + ...
  state + state + state = state
  ```
* Fork and join:
  ```
  state = contract state || contract state || ...
  state || state || state = state
  ```
  On a fork, for each part an assume-guarantee (AG) contract has to be provided.
  The syntax of AG contract is `@contract(parameters)`.
  AG contracts are defined separately by extending the class `AssumeGuaranteeContract` in [verification/spec/contract.py](pgcd/nodes/verification/spec/contract.py).
  For independent robots, the AG contracts only need to specify footprints and we offer the syntax: `{ contstraints over fpx fpy fpz }`.
  A point is included in the footprint if the constraints evaluate to true when `fpx`, `fpy`, and `fpz` are replaced with the point's coordinates.

The [test folder](pgcd/nodes/verification/test/) contains examples (files with name starting with `xp_`).

#### Model of the Environment

The environment is models using [components](pgcd/nodes/verification/spec/component.py).
A component can be either a `Process` or an `Obstacle`.
Processes are active and executes programs while obstacle are passive.

Components are linked together through a parent/child relationship.
The links specifies how the relative coordinate systems are related.
We provide a [World](pgcd/nodes/verification/test/experiments_setups.py) component with frame shifts given as parameters as placeholder for the room in which the robots are placed.


#### Model of the Robots

Robots are modeled as [Process](pgcd/nodes/verification/spec/component.py).
A process has variables (input and output), a footprint, a default AG contract (invariant), motion primitives.
A process can be connected to other processes, e.g., arm attached on top of the cart.

Examples of robots models are:
[cart](pgcd/nodes/verification/test/cart.py),
[simple arm](pgcd/nodes/verification/test/arm.py),
[Franka Emika Panda arm](pgcd/nodes/verification/test/franka.py),
[static process](pgcd/nodes/verification/test/static_process.py).

#### Motion Primitives Specification

Motion primitive are specified by extending the [MotionPrimitive](pgcd/nodes/verification/spec/motion.py) class.
Motion primitive extends AG contracts.
To create motion primitives which may have parameters, a corresponding `MotionPrimitiveFactory` needs to be created.
Examples of motion primitives specifications can be found along the robot models listed above.

Currently, we assume the motion primitive specifications but do not verify this.
There are plenty of work which focus on that and can be used there.
Our focus is how specify the communication and motions to help compositional verification.

#### Calling the Verifier

The simplest it to extend the class [XpTestHarness](pgcd/nodes/verification/test/experiments_setups.py).
Then calling `self.check(choreography, world, contracts, programs)` setup the verifier and calls it.
The arguments are
* `choreography`: the choreographic specification as a string,
* `world`: the environment specification a `Component`,
* `contracts`: a list of AG contracts (names of the classes),
* `programs`: a map from process IDs to programs.

`XpTestHarness` does the following:
1. parse the choreography and perform some syntactic checks on it,
2. analyzing the choreography in more details (checking the times, (un)interruptible motions, process partition in forks, etc.) and generates the verification conditions (VCs),
3. discharge the VCs,
4. refinement check of the programs against the choreography's endpoints projections.

