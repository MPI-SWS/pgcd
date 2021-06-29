#!/usr/bin/env python3

# std lib
import sys
import importlib
import queue
import threading
import time
import inspect
# ros
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from rclpy.duration import Duration
import tf2_ros
#from tf2_geometry_msgs import PoseStamped, Vector3Stamped, PointStamped, WrenchStamped
from geometry_msgs.msg import Pose, Vector3, Point, Wrench
#from geometry_msgs.msg import PoseStamped, Vector3Stamped, PointStamped, WrenchStamped
from std_msgs.msg import String
# our
from tf_updater import TFUpdater
from interpreter.interpreter import Interpreter
from interpreter.status import InterpreterStatus
from interpreter.communication_utils import *
from recovery.recovery_manager import RecoveryManager
from verification.choreography.parser_chor import ChoreographyParser
from verification.spec.component import World
from verification.spec.env import Env
from pgcd.msg import ErrorStamped, CompensationStamped

class Component(Node,Interpreter,TFUpdater):

    def __init__(self):
        Node.__init__(self, "pgcd_comp", allow_undeclared_parameters = True, automatically_declare_parameters_from_overrides = True)
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
        self.tfBuffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.tf2_setup(self.robot)
        #self.tfBuffer.registration.print_me()
        # recovery
        false = rclpy.parameter.Parameter('recovery',rclpy.parameter.Parameter.Type.BOOL, False)
        if self.get_parameter_or('recovery', false).value:
            self.choreo_path = self.get_parameter('choreography_location')._value
        else:
            self.choreo_path = None

    def message_callback(self, msg):
        # make sure there is an header, get the sender (frame), convert to local frame, put in queue
        rate = self.create_rate(5)
        #while not rclpy.is_shutdown():
        try:
            while True:
                try:
                    msg = self.tfBuffer.transform(msg, self.id)
                    #TODO add transform for pgcd.msg types: recurse through the defs and convert what we know how to convert
                    break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print("PGCD message conversion 1 ", str(e))
                    rate.sleep()
        except Exception as e:
            rclpy.logging._root_logger.log("PGCD message " + str(msg) + ": " + str(e), LoggingSeverity.ERROR)
        rate.destroy()
        try:
            self.receive_from[msg.header.frame_id].put_nowait(msg)
        except queue.Full:
            self.get_logger().warning('dropping message due to full queue')

    def ack_callback(self, msg):
        try:
            self.receive_from[msg.data].put_nowait(msg)
        except queue.Full:
            self.get_logger().warning('dropping message due to full queue')

    def setup_communication(self):
        ack = set()
        topics = set()
        # subscriptions
        rclpy.logging._root_logger.log("PGCD sub /" + self.id + "/Ack", LoggingSeverity.INFO)
        self.create_subscription(String, "/" + self.id + "/Ack", self.ack_callback, 1)
        for name, msg_name in get_receive_info(self.program):
            topic = "/" + self.id + "/" + msg_name
            if not topic in topics:
                topics.add(topic)
                rclpy.logging._root_logger.log("PGCD sub " + topic + " " + str(self.msg_types[msg_name]), LoggingSeverity.INFO)
                self.create_subscription(self.msg_types[msg_name], topic, self.message_callback, 1)
            if name not in ack:
                self.receive_from[name] = queue.Queue(10)
                self.send_to[name] = {}
                rclpy.logging._root_logger.log("PGCD pub /" + name + "/Ack", LoggingSeverity.INFO)
                self.send_to[name]["Ack"] = self.create_publisher(String, "/" + name + "/Ack", 1) #to emulate synchronous communication
                ack.add(name)
        # publishers
        for name, msg_name in get_send_info(self.program):
            if name not in self.send_to:
                self.send_to[name] = {}
            if name not in ack:
                self.receive_from[name] = queue.Queue(10)
                ack.add(name)
            topic = "/" + name + "/" + msg_name
            if not topic in topics:
                topics.add(topic)
                rclpy.logging._root_logger.log("PGCD pub " + topic + " " + str(self.msg_types[msg_name]), LoggingSeverity.INFO)
                self.send_to[name][msg_name] = self.create_publisher(self.msg_types[msg_name], topic, 1)
        # tf2 stuff
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.tf2_timer_callback)


    def getEnv(self):
        # get the environement (spec of processes) so we could compute the recovery
        processesIDs = self.get_parameter('env_processes_ids')._value
        processesModule = self.get_parameter('env_processes_module')._value
        processesClasses = self.get_parameter('env_processes_class')._value
        w = World()
        i = 0 # index for the world
        for (name,mod,cls) in zip(processesIDs, processesModule, processesClasses):
            try:
                rclpy.logging._root_logger.log("PGCD getting module " + str(mod), LoggingSeverity.INFO)
                m = importlib.import_module(mod)
                assert inspect.ismodule(m)
                #rclpy.logging._root_logger.log("PGCD got module " + str(inspect.getmembers(m)), LoggingSeverity.INFO)
                rclpy.logging._root_logger.log("PGCD getting class " + str(cls), LoggingSeverity.INFO)
                obj = getattr(m, cls)
                assert inspect.isclass(obj)
                obj(name, parent = w, index = i) # create the spec, add to the world
                i += 1
            except KeyError as e:
                rclpy.logging._root_logger.log("PGCD could not find spec " + cls + " of " + name, LoggingSeverity.ERROR)
        return Env(w, [])

    def setup_recovery(self):
        rclpy.logging._root_logger.log("PGCD setup for recovery" + self.id, LoggingSeverity.INFO)
        # create recovery manager
        with open(self.choreo_path, 'r') as content_file:
            choreo_src = content_file.read()
        env = self.getEnv()
        choreo = ChoreographyParser(env).parse(choreo_src, check = False)
        self.recoveryMgr = RecoveryManager(self, choreo)
        # create publishers
        self.recoveryMgr.error_pub = self.create_publisher(ErrorStamped, "/pgcd/error", 1)
        self.recoveryMgr.compensation_pub = self.create_publisher(CompensationStamped, "/pgcd/compensation", 1)
        # set callback for failure and compensation
        self.create_subscription(ErrorStamped, "/pgcd/error", self.recoveryMgr.failure_callback, 1)
        self.create_subscription(CompensationStamped, "/pgcd/compensation", self.recoveryMgr.comp_callback,  len(env.allProcesses()))

    def execute_prog(self):
        with open(self.prog_path, 'r') as content_file:
            program = content_file.read()
        self.parse(program)
        self.setup_communication()
        if self.choreo_path != None:
            self.setup_recovery()
        while self.status != InterpreterStatus.TERMINATED:
            self.execute()
            if self.status == InterpreterStatus.ERROR:
                # notify the others
                rclpy.logging._root_logger.log("PGCD got failure " + self.id, LoggingSeverity.INFO)
                self.recoveryMgr.failure()
                # start recovery
                rclpy.logging._root_logger.log("PGCD start recovery " + self.id, LoggingSeverity.INFO)
                self.recoveryMgr.startRecovery()
            elif self.status == InterpreterStatus.INTERRUPTED:
                # start recovery
                rclpy.logging._root_logger.log("PGCD start recovery " + self.id, LoggingSeverity.INFO)
                self.recoveryMgr.startRecovery()
            else:
                rclpy.logging._root_logger.log("PGCD terminated " + self.id + " with " + str(self.status), LoggingSeverity.INFO)
                assert self.status == InterpreterStatus.TERMINATED
                if self.recoveryMgr.isRecovering:
                    self.recoveryMgr.restoreProgram()
        self.evt.set()

    def run(self, executor=None):
        if (executor == None):
            executor = MultiThreadedExecutor()
        rclpy.logging._root_logger.log("PGCD creating " + self.id + " starting interpreter", LoggingSeverity.INFO)
        self.evt = threading.Event()
        t = threading.Thread(name='interpreter', target=self.execute_prog)
        t.start()
        while not self.evt.is_set():
            rclpy.spin_once(self, executor=executor) #does that make sense with mlutithreaded exec?
        t.join()

def main(args=None):
    rclpy.init(args=args)
    comp = Component()
    comp.run()
    comp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
