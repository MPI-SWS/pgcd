#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
import tf_updater
from interpreter import Interpreter
import importlib
from std_msgs.msg import String
import queue

class Component(Node,Interpreter,TFUpdater):

    def __init__(self):
        Node.__init__(rclpy.get_name())
        Interpreter.__init__(self, rclpy.get_name())
        # program
        self.id = rclpy.get_name()[1:]
        self.prog_path = rclpy.get_param('~program_location') + self.id + '.rosl'
        # hardware
        module = importlib.import_module(rclpy.get_param('~object_module_name'))
        class_ = getattr(module, rclpy.get_param('~object_class_name'))
        self.robot = class_()
        # runtime
        self.tf2_setup(self.robot)
        self.setup_communication()
        self.tfBuffer = tf2_ros.Buffer()

    def message_callback(self, msg):
        # make sure there is an header, get the sender (frame), convert to local frame, put in queue
        trans = self.tfBuffer.lookup_transform(msg.source_frame, self.id, rclpy.Time())
        #TODO convert the message content
        self.get_logger().warning('TODO frame conversion')
        try:
            self.receive_from[msg.source_frame].put_nowait(msg)
        except queue.Full:
            self.get_logger().warning('dropping message due to full queue')
    
    def ack_callback(self, msg):
        try:
            self.receive_from[msg.data].put_nowait(msg)
        except queue.Full:
            self.get_logger().warning('dropping message due to full queue')
    
    def setup_communication(self):
        ack = set()
        # subscriptions
        self.create_subscription(String, "/" + self.id + "/Ack", self.ack_callback, 1)
        for name, msg_name in self.get_receive_info():
            self.create_subscription(self.msg_types[msg_name], "/" + self.id + "/" + msg_name, self.message_callback, 1)
            if name not in ack:
                self.receive_from[name] = queue.Queue(10)
                self.send_to[name] = {}
                self.send_to[name]["Ack"] = self.create_publisher(String, "/" + name + "/Ack", 1) #to emulate synchronous communication
        # publishers
        for name, msg_name in self.get_send_info():
            if name not in self.send_to:
                self.send_to[name] = {}
            self.send_to[name][msg_name] = self.create_publisher(self.msg_types[msg_name], "/" + name + "/" + msg_name, 1)
        # tf2 stuff
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.tf2_timer_callback)

    def execute_prog(self):
        with open(self.prog_path, 'r') as content_file:
            program = content_file.read()
        self.execute(program)

    def run(self, executor=MultiThreadedExecutor()):
        rclpy.spin(self, executor=executor)
        self.execute_prog()

if __name__ == '__main__':
    rclpy.init(args=args)
    #rclpy.init_node('fixed_tf2_broadcaster')
    tfb = Component()
    tfb.run()
    tbf.destroy_node()
    rclpy.shutdown()
