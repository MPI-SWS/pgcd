#!/usr/bin/env python3

import rospy
import _thread
import tf_updater
from executor import Executor
import importlib

class Component(Executor):

    def __init__(self):
        Executor.__init__(self, rospy.get_name())
        self.id = rospy.get_name()[1:]
        self.components = {}
        self.prog_path = rospy.get_param('~program_location') + self.id + '.rosl'

        module = importlib.import_module(rospy.get_param('~object_module_name'))
        class_ = getattr(module, rospy.get_param('~object_class_name'))
        self.robot = class_()

        tf_updater.TFUpdater(self.robot)
        self.execute_prog()
    
    def execute_prog(self):
        # try:
            with open(self.prog_path, 'r') as content_file:
                program = content_file.read()
            self.execute(program)
        # except Exception as e:
        #     print(str(e))

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = Component()
    rospy.spin()
