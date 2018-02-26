#!/usr/bin/env python3

import execnet
import sys

print(sys.version)


def call_python_version(Version, Module, Function, ArgumentList):
    gw = execnet.makegateway("popen//python=python%s" % Version)
    channel = gw.remote_exec("""
        from %s import %s as the_function
        channel.send(the_function(*channel.receive()))
    """ % (Module, Function))
    channel.send(ArgumentList)
    #return channel.receive()


result = call_python_version("2.7", "main", "start_ros_nodes", [])
