#!/usr/bin/env python

import rospy
import tf2_msgs.msg

import encasp_2r2p as en
import tf_updater
from executor import Executor


class Component(Executor):

    components = {}

    def __init__(self):
        Executor.__init__(self, rospy.get_param('~comp'))
        self.id = rospy.get_param('~comp')
        self.prog_path = rospy.get_param('~program_location') + self.id + '.rosl'
        try:
            self.parent = rospy.get_param('~parent')
        except KeyError:
            self.parent = "world"
        self.robot = en.robot_arm()
        self.init_parts()
        self.execute_prog()

    def init_parts(self):
        for l in self.robot.list:
            self.components[l.getName()] = tf_updater.TFUpdater(self.id, self.parent, l)

    def execute_prog(self):
        with open(self.prog_path, 'r') as content_file:
            program = content_file.read()
        self.execute(program)


if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = Component()
    rospy.spin()
