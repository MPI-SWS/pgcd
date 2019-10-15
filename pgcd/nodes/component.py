#!/usr/bin/env python3

import rclpy
import _thread
import tf_updater
from executor import Executor
import importlib

class Component(Executor):

    def __init__(self):
        Executor.__init__(self, rclpy.get_name())
        self.id = rclpy.get_name()[1:]
        self.prog_path = rclpy.get_param('~program_location') + self.id + '.rosl'

        module = importlib.import_module(rclpy.get_param('~object_module_name'))
        class_ = getattr(module, rclpy.get_param('~object_class_name'))
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
    rclpy.init_node('fixed_tf2_broadcaster')
    tfb = Component()
    rclpy.spin()
