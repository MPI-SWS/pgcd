import geometry_msgs.msg
import rclpy
import tf2_py
import tf2_ros
import tf2_msgs.msg
import random
import sympy as sp


class TFUpdater:

    def __init__(self):
        self.parent_ids = []
        self.frames_ids = []
        self.updater_func_names = []

    def tf2_setup(self, robot):
        try:
            k = 1
            while self.has_parameter('frame_' + str(k) + '_parent'):
                parent_name = self.get_parameter('frame_' + str(k) + '_parent')._value
                frame_name = self.get_parameter('frame_' + str(k))._value
                updt_func_name = str(self.get_parameter('frame_' + str(k) + '_updater')._value)
                k += 1
                self.parent_ids.append(parent_name)
                self.frames_ids.append(frame_name)
                self.updater_func_names.append(getattr(robot, updt_func_name))
        except Exception as e:
            print("got", e)
            pass
        #self.pub_tf = rclpy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=4)
        self.pub_tf = tf2_ros.StaticTransformBroadcaster(self)

    #TODO avoid the sympy evaluation!!
    def tf2_timer_callback(self):
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
        x = [[matrix[j, i] for i in range(0, 4)] for j in range(0, 4)]
        q = tf2_py.transformations.quaternion_from_matrix(x) #FIXME
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        #publish
        #tfm = tf2_msgs.msg.TFMessage([t])
        #self.pub_tf.publish(tfm)
        self.pub_tf.sendTransform(t)
