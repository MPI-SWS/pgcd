#!/usr/bin/env python
import random

import geometry_msgs.msg
import geometry_msgs.msg
import rospy
import tf2_msgs.msg
import turtlesim.srv
import math
import math
import turtlesim.srv
import random
from ast_executor import Executor
from rfccc.msg import MoveToPosition, Rotate
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from ast_executor import Executor
import tf


class FixedComponent(Executor):
    strToMsg = {'Rotate': Rotate, 'MoveToPosition': MoveToPosition}

    def __init__(self):
        Executor.__init__(self, rospy.get_param('~comp'))
        self.fix_name = rospy.get_param('~comp')
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        try:
            self.parent = rospy.get_param('~parent')
            self.x = rospy.get_param('~x')
            self.y = rospy.get_param('~y')
            self.z = rospy.get_param('~z')
        except:
            self.parent = "world"
        self.timer = rospy.Timer(rospy.Duration(nsecs=1), self.set_up_broadcaster)
        self.yaw = 0
        self.pitch = 0
        self.roll = 0

    def set_up_broadcaster(self, event):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.parent
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.fix_name
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        # q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx')
        # RADIANS
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
    if tfb.fix_name == 'arm':
        program = '''
            receive(m_Idle){(msg_Rotate, angles, { m_Rotate(angles) })}; 
            angles := {"yaw": 3.14159, "pitch":0, "roll":0};  
            send(id_tool, msg_Rotate, angles)         
        '''
    else:
        program = '''
            receive(m_Idle){ (msg_Rotate, angles, { m_Rotate(angles) }) };
            position := {"x": 10, "y":7, "z":0 };  
            send(id_turtle1, msg_MoveToPosition, position)            
        '''
    tfb.execute(program)
    rospy.spin()
    # tfb.timer.shutdown()
