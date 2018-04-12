import geometry_msgs.msg
import geometry_msgs.msg
import rospy
import tf
import tf2_msgs.msg
import random
import numpy as np
from cartandarm import cart, arm
from carrier import carrier
import sympy as sp


class TFUpdater:

    def __init__(self, robot):
        self.robot = robot
        self.parent_ids = []
        self.frames_ids = []
        self.updater_func_names = []

        try:
            k = 1
            while True:
                parent_name = rospy.get_param('~frame_' + str(k) + '_parent')
                frame_name = rospy.get_param('~frame_' + str(k))
                updt_func_name = rospy.get_param('~frame_' + str(k) + '_updater')
                k += 1
                self.parent_ids.append(parent_name)
                self.frames_ids.append(frame_name)
                self.updater_func_names.append(getattr(self.robot, updt_func_name))
        except Exception as e:
            pass

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=4)
        self.timer = rospy.Timer(rospy.Duration(nsecs=100000000), self.set_up_broadcaster)

    def set_up_broadcaster(self, _):
        for parent, frame, updater in zip(self.parent_ids, self.frames_ids, self.updater_func_names):
            self.updateMatrix(sp.N(updater()), frame, parent)

    def updateMatrix(self, matrix, id_frame, parent_frame):
        #print("Updating: ", id_frame, parent_frame, matrix)
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent_frame
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = id_frame
        
        position = matrix
        x = [[matrix[j, i] for i in range(0, 4)] for j in range(0, 4)]

        t.transform.translation.x = position[0, 3]
        t.transform.translation.y = position[1, 3]
        t.transform.translation.z = position[2, 3]
        # q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') RADIANS
        q = tf.transformations.quaternion_from_matrix(x)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)
