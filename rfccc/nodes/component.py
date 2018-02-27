#!/usr/bin/env python3

import rospy
import tf2_msgs.msg

import encasp_2r2p as en
import tf_updater
from executor import Executor
import sympy as sp

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
        self.robot.set_angle_base(0, 0,3.14159 / 4)
        self.robot.set_angle_elbow(0,0,-3.14159 / 4)
        self.robot.calculateEndPosition()
        self.init_parts()
        self.execute_prog()

    def init_parts(self):
        list_of_acts = self.robot.linkage.getListOfActuators()
        self.components[self.robot.list[0].getName()] = tf_updater.TFUpdater(self.id, self.parent, self.robot.list[0])
        for i in range(1, len(self.robot.list)):
            parent = self.id + "_" + self.robot.list[i-1].getName()
            self.components[self.robot.list[i].getName()] = tf_updater.TFUpdater(self.id, parent,
                                                                                 self.robot.list[i])

    def execute_prog(self):
        with open(self.prog_path, 'r') as content_file:
            program = content_file.read()
        self.execute(program)


if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = Component()
    rospy.spin()
