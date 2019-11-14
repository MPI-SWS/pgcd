#!/usr/bin/env python3

# std lib
import sys
import importlib
import queue
# ros
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.duration import Duration
import tf2_ros
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, WrenchStamped
from std_msgs.msg import String
# our
from interpreter.interpreter import Interpreter
from tf_updater import TFUpdater 

class Component(Node,Interpreter,TFUpdater):

    def __init__(self):
        Node.__init__(self, "pgcd_comp",
                allow_undeclared_parameters = True,
                automatically_declare_parameters_from_overrides = True)
        Interpreter.__init__(self, self.get_name())
        TFUpdater.__init__(self)
        # program
        self.id = self.get_name()
        self.prog_path = self.get_parameter('program_location')._value + self.id + '.rosl'
        rclpy.logging._root_logger.log("PGCD creating " + self.id + " running " + self.prog_path, LoggingSeverity.INFO)
        # hardware
        module = importlib.import_module(self.get_parameter('object_module_name')._value)
        class_ = getattr(module, self.get_parameter('object_class_name')._value)
        self.robot = class_()
        # runtime
        self.tf2_setup(self.robot)
        self.setup_communication()
        self.tfBuffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def message_callback(self, msg):
        # make sure there is an header, get the sender (frame), convert to local frame, put in queue
        try:
            dummy = msg.header.frame_id #stupid way of checking if it is stamped
            rate = rclpy.Rate(10.0)
            while not rclpy.is_shutdown():
                try:
                    msg = self.tfBuffer.transform(msg, self.id) #TODO add transform for pgcd.msg types: recurse through the defs and convert what we know how to convert
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rate.sleep()
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

    def run(self, executor=None):
        if (executor == None):
            executor = MultiThreadedExecutor()
        rclpy.spin(self, executor=executor)
        self.execute_prog()

def main(args=None):
    rclpy.init(args=args)
    #rclpy.init_node('fixed_tf2_broadcaster')
    comp = Component()
    comp.run()
    comp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
