#!/usr/bin/env python

import roslaunch
import xml.etree.ElementTree
import os
from choreography.parser_chor import *

def start_ros_nodes():
    package = 'rfccc'
    broadcaster_comp = 'component.py'

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    e = xml.etree.ElementTree.parse(os.getcwd() + '/start.launch').getroot()
    for atype in e.findall('group'):
        node = roslaunch.core.Node(package, broadcaster_comp, name=atype.get('ns'), respawn=False, output="screen")
        launch.launch(node)
        Choreography.initialized_components.add(atype.get('ns'))

    input()

start_ros_nodes()
