import geometry_msgs.msg
import rclpy
from rclpy.logging import LoggingSeverity
import tf2_ros
import tf2_msgs.msg
import random
import sympy as sp
import math


class TFUpdater:

    def __init__(self):
        self.parent_ids = []
        self.frames_ids = []
        self.updater_func_names = []

    def tf2_setup(self, robot):
        k = 1
        while self.has_parameter('frame_' + str(k) + '_parent'):
            parent_name = self.get_parameter('frame_' + str(k) + '_parent')._value
            frame_name = self.get_parameter('frame_' + str(k))._value
            updt_func_name = str(self.get_parameter('frame_' + str(k) + '_updater')._value)
            k += 1
            self.parent_ids.append(parent_name)
            self.frames_ids.append(frame_name)
            self.updater_func_names.append(getattr(robot, updt_func_name))
        self.pub_tf = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_timer = self.create_timer(1.0, self.tf2_timer_callback) # seems too fast ?!

    #TODO avoid the sympy evaluation!!
    def tf2_timer_callback(self):
        rclpy.logging._root_logger.log("PGCD tf2_timer_callback", LoggingSeverity.DEBUG)
        for parent, frame, updater in zip(self.parent_ids, self.frames_ids, self.updater_func_names):
            self.updateMatrix(sp.N(updater()), frame, parent)

    def updateMatrix(self, matrix, id_frame, parent_frame):
        #print("Updating: ", id_frame, parent_frame, matrix)
        t = geometry_msgs.msg.TransformStamped()
        #header
        t.header.frame_id = parent_frame
        #t.header.stamp = rclpy.time.Time() #.now()
        t.child_frame_id = id_frame
        #translation
        t.transform.translation.x = float(matrix[0, 3])
        t.transform.translation.y = float(matrix[1, 3])
        t.transform.translation.z = float(matrix[2, 3])
        #rotation
        w = math.sqrt(1.0 + float(matrix[0,0]) + float(matrix[1,1]) + float(matrix[2,2])) / 2.0
        x = (float(matrix[2,1]) - float(matrix[1,2])) / (4*w)
        y = (float(matrix[0,1]) - float(matrix[2,0])) / (4*w)
        z = (float(matrix[1,0]) - float(matrix[0,1])) / (4*w)
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        #publish
        self.pub_tf.sendTransform(t)
