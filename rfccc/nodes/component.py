#!/usr/bin/env python3

import rospy

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
            self.x = rospy.get_param('~x')
            self.y = rospy.get_param('~y')
            self.z = rospy.get_param('~z')
        except KeyError as e:
            print(str(e))
            self.parent = "world"
            self.x = 0
            self.y = 0
            self.z = 0
        self.robot = en.robot_arm(self.x, self.y, self.z)
        self.robot.set_angle_base(0, 0, 0)
        self.robot.set_angle_elbow(0, 0, 0)
        self.robot.calculateEndPosition()
        self.init_parts()
        self.execute_prog()

    def init_parts(self):
        self.components[self.robot.list[0].getName()] = tf_updater.TFUpdater(self.id, self.parent, self.robot.list[0], self.robot.list)
        for i in range(1, len(self.robot.list)):
            parent = self.id + "_" + self.robot.list[i-1].getName()
            self.components[self.robot.list[i].getName()] = tf_updater.TFUpdater(self.id, parent,
                                                                                 self.robot.list[i], self.robot.list)

    def execute_prog(self):
        try:
            with open(self.prog_path, 'r') as content_file:
                program = content_file.read()
            self.execute(program)
        except Exception as e:
            print(str(e))

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = Component()
    rospy.spin()
