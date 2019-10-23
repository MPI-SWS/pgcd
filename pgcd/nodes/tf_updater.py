import geometry_msgs.msg
import rclpy
import tf
import tf2_msgs.msg
import random
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
                parent_name = rclpy.get_param('~frame_' + str(k) + '_parent')
                frame_name = rclpy.get_param('~frame_' + str(k))
                updt_func_name = rclpy.get_param('~frame_' + str(k) + '_updater')
                k += 1
                self.parent_ids.append(parent_name)
                self.frames_ids.append(frame_name)
                self.updater_func_names.append(getattr(self.robot, updt_func_name))
        except Exception as e:
            pass

        self.pub_tf = rclpy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=4)
        self.timer = rclpy.Timer(rclpy.Duration(nsecs=100000000), self.set_up_broadcaster)

    def set_up_broadcaster(self, _):
        for parent, frame, updater in zip(self.parent_ids, self.frames_ids, self.updater_func_names):
            self.updateMatrix(sp.N(updater()), frame, parent)

    def updateMatrix(self, matrix, id_frame, parent_frame):
        #print("Updating: ", id_frame, parent_frame, matrix)
        t = geometry_msgs.msg.TransformStamped()
        #header
        t.header.frame_id = parent_frame
        t.header.stamp = rclpy.Time.now()
        t.child_frame_id = id_frame
        #translation
        t.transform.translation.x = matrix[0, 3]
        t.transform.translation.y = matrix[1, 3]
        t.transform.translation.z = matrix[2, 3]
        #rotation
        x = [[matrix[j, i] for i in range(0, 4)] for j in range(0, 4)]
        q = tf.transformations.quaternion_from_matrix(x)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        #publish
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)