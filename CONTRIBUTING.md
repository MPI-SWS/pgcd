# ROSL - Concurrent Control Code in ROS with resources and frames tracking

ROSL is a programming language for ROS nodes.

## Project description

Currently the main functionalities of ROSL are:
* Communication between ROS nodes (ROS message & ROS topics)
* Execute motion primitive
* Tracking frames (TF2 library)

## Project setup

To run ROSL you need to:

* install ROS ( reccomended ROS Kinetic: http://wiki.ros.org/kinetic/Installation)
* install tf2 library with turtlesim example (http://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2)
* download this project
* To add the project workspace to your ROS environment you need to source the generated setup file: 

```
$ . path-to-project-devel/devel/setup.bash
```

### To run the project we reccomend PyCharm.

Install python 2.7.9 and packages:
* PyYAML
* enum34
* genpy
* numpy
* ply
* rospy
* rospkg
* rosdistro
* rosdep
* tf
* tf2-ros

