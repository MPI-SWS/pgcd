#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import tf2_ros
import tf_updater
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, WrenchStamped
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
        self.tfBuffer = tf2_ros.Buffer(rclpy.Duration(600.0))
        self.tf2_listener = tf2_ros.TransformListener(tfBuffer)

    def message_callback(self, msg):
        # make sure there is an header, get the sender (frame), convert to local frame, put in queue
        try:
            dummy = msg.header.frame_id #stupid way of checking if it is stamped
            rate = rclpy.Rate(10.0)
            while not rclpy.is_shutdown():
                msg = self.tfBuffer.transform(msg, self.id) #TODO add transform for pgcd.msg types: recurse through the defs and convert what we know how to convert
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
        except AttributeError:
            pass
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
