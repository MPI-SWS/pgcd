import geometry_msgs.msg
import geometry_msgs.msg
import rospy
import tf
import tf2_msgs.msg
import random
import numpy as np
from cartandarm import cart, arm


class TFUpdater:

    def __init__(self, _id, parent, robot):
        self.id = _id
        self.parent = parent
        self.debug = True
        self.robot = robot
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(nsecs=100000000), self.set_up_broadcaster)
        #else:
        #    self.rand = random.randint(2, 5)
        #    self.timer = rospy.Timer(rospy.Duration(nsecs=1000000), self.set_up_default_broadcaster)

    def set_up_default_broadcaster(self, _):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.parent
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.id

        t.transform.translation.x = self.rand * 0.1
        t.transform.translation.y = 2 - 0.3 * self.rand * 0.2
        t.transform.translation.z = self.rand * 0.5
        # print(x)
        # q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') RADIANS
        q = tf.transformations.quaternion_from_matrix(np.ones([4, 4]))

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)


    def set_up_broadcaster(self, _):
        #if self.id_matrix == 0:
        #    matrix = self.matrix.getConfigurationMatrixTurntable()
        #elif self.id_matrix == 1:
        #    matrix = self.matrix.getConfigurationMatrixCantilever()
        #elif self.id_matrix == 2:
        #    matrix = self.matrix.getConfigurationMatrixAnchorPoint()
        #elif self.id_matrix == 3:
        #    matrix = self.matrix.getConfigurationMatrixGripper()
        #elif self.id_matrix == 4:
        #    matrix = self.matrix.getConfigurationMatrixCart()

        print( "broadcaster: type=", type(self.robot) )
        if type( self.robot ) == cart:
            self.updateMatrix( self.robot.getConfigurationMatrixCart(), "cart_frame", "world" )
        else:
            self.updateMatrix( self.robot.getConfigurationMatrixTurntable(), "frame_turntable", "cart_frame" )
            self.updateMatrix( self.robot.getConfigurationMatrixCantilever(), "frame_cantilever", "frame_turntable" )
            self.updateMatrix( self.robot.getConfigurationMatrixAnchorPoint(), "frame_anchorpoint", "frame_cantilever" )
            self.updateMatrix( self.robot.getConfigurationMatrixGripper(), "frame_gripper", "frame_anchorpoint" )

    def updateMatrix( self, matrix, id_frame, parent_frame ):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent_frame

        t.header.stamp = rospy.Time.now()
        t.child_frame_id = id_frame
        
        position = matrix


        x = [[matrix[j, i] for i in range(0, 4)] for j in range(0, 4)]

        #print( "broadcaster:", x )
        t.transform.translation.x = position[0, 3]
        t.transform.translation.y = position[1, 3]
        t.transform.translation.z = position[2, 3]
        # print(x)
        # q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx') RADIANS
        q = tf.transformations.quaternion_from_matrix(x)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

