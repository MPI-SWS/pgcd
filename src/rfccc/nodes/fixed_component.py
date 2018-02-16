#!/usr/bin/env python

import geometry_msgs.msg
import geometry_msgs.msg
import rospy
import tf
import tf2_msgs.msg
from rfccc.msg import MoveToPosition, Rotate

from executor import Executor


class FixedComponent(Executor):
    strToMsg = {'Rotate': Rotate, 'MoveToPosition': MoveToPosition}

    def __init__(self):
        Executor.__init__(self, rospy.get_param('~comp'))
        self.id = rospy.get_param('~comp')
        self.prog_path = rospy.get_param('~program_location') + self.id + '.rosl'
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        try:
            self.parent = rospy.get_param('~parent')
            self.x = rospy.get_param('~x')
            self.y = rospy.get_param('~y')
            self.z = rospy.get_param('~z')
        except:
            self.parent = "world"
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.timer = rospy.Timer(rospy.Duration(nsecs=10), self.set_up_broadcaster)
        with open(self.prog_path, 'r') as content_file:
            program = content_file.read()
        self.execute(program)

    def parse_in_thread(self, code):
        pass

    def set_up_broadcaster(self, _):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.parent
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        # q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') RADIANS
        q = tf.transformations.quaternion_from_euler(self.yaw, self.pitch, self.roll, 'rzyx')
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)


if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedComponent()
    rospy.spin()
