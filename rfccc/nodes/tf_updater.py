import geometry_msgs.msg
import geometry_msgs.msg
import rospy
import tf
import tf2_msgs.msg
import numpy as np


class TFUpdater:

    def __init__(self, id, parent, component):
        self.id = id
        self.parent = parent
        self.debug = True
        self.component = component
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(nsecs=5), self.set_up_broadcaster)


    def set_up_broadcaster(self, _):
        if self.debug:
            print(self.component.getName(), self.parent)
            self.debug = False
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.parent
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.id + "_" + self.component.getName()
        #print(self.component.getName())
        matrix = self.component.getPositionMatrix()
        position = self.component.getCoordinates()
        x = [[matrix[j, i] for i in range(0, 4)] for j in range(0, 4)]
        #print(self.component.getName(), matrix)
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        # print(x)
        # q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') RADIANS
        q = tf.transformations.quaternion_from_matrix(x)
        #q = tf.transformations.quaternion_from_euler(ax, ay, az, 'rxyz')
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)
