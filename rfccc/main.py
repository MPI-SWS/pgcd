#!/usr/bin/env python
import rospy
import roslaunch


def start_ros_nodes():
    print("start")
    package = 'rfccc'
    broadcaster_comp = 'component.py'
    node2 = roslaunch.core.Node(package, broadcaster_comp, name="arm_fix", respawn=False, output="screen")
    node3 = roslaunch.core.Node(package, broadcaster_comp, name="tool_fix", respawn=False, output="screen")

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    launch.launch(node2)
    #launch.launch(node3)
    print("nodes started")
    input()

start_ros_nodes()
