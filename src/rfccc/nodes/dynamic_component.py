#!/usr/bin/env python
import rospy

# Because of transformations
import tf
import tf2_msgs.msg
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg
import math
import turtlesim.srv
import random
from code_exec import Executor
from rfccc.msg import MoveToPosition, Rotate
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

'''
Every node has to broadcast its position in the 
'''


class DynamicComponent(Executor):
    strToMsg = {'Rotate': Rotate, 'MoveToPosition': MoveToPosition}

    def __init__(self):
        Executor.__init__(self)
        self.turtle_name = rospy.get_param('~turtle')
        self.parent = "world"
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel' % self.turtle_name, Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/%s/pose' % self.turtle_name, Pose, self.callback)
        self.pose = Pose()
        self.wait = True
        self.rate = rospy.Rate(10)
        self.set_up_broadcaster()

    def attribute(self, name):
        return getattr(self, name)

    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def visit_send(self, component, label, expression):
        if label == 'MoveToPosition':
            self.pub = rospy.Publisher("/" + component, MoveToPosition, queue_size=10)
            assert 'x' in expression.keys()
            assert 'y' in expression.keys()
            assert 'z' in expression.keys()
            message = MoveToPosition(x=expression['x'],
                                     y=expression['y'],
                                     z=expression['z'])
            self.send_msg(message)

        elif label == 'Rotate':
            self.pub = rospy.Publisher("/" + component, Rotate, queue_size=10)
            assert 'yaw' in expression.keys()
            assert 'pitch' in expression.keys()
            assert 'roll' in expression.keys()
            message = Rotate(yaw=expression['yaw'],
                             pitch=expression['pitch'],
                             roll=expression['roll'])
            self.send_msg(message)

    def send_msg(self, message):
        while self.pub.get_num_connections() == 0:
            pass
            print 'WAITING..'
            rospy.sleep(0.1)
        self.pub.publish(message)

    def visit_receive(self, motion, actions):
        self.wait = True
        for action in actions:
            assert action['msg'] in self.strToMsg.keys()
            self.sub = rospy.Subscriber("/" + self.turtle_name, self.strToMsg[action['msg']],
                                        self.msg_store_var,
                                        callback_args={
                                            "var_name": action['var'],
                                            "program": action['stmt']
                                        },
                                        queue_size=100)
        while self.wait:
            self.do_action(motion.value, motion.exps)

    def msg_store_var(self, msg, args):
        print "Exec msg..."
        if self.wait:
            self.sub.unregister()
            if args['var_name'] in self.variables.keys():
                self.variables[args['var_name']] = {'x': msg.x, 'y':msg.y, 'z': msg.z}
            else:
                setattr(self, args['var_name'], {'x': msg.x, 'y':msg.y, 'z': msg.z})
            args['program'].accept(self)
            self.wait = False

    def move_to_pos(self, pos):
        if hasattr(self, 'sub'):
            self.sub.unregister()
        self.wait = False
        goal_pose = Pose()
        goal_pose.x = pos.x
        goal_pose.y = pos.y
        distance_tolerance = 0.1
        vel_msg = Twist()
        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:
            vel_msg.linear.x = 0.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            # print sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def do_action(self, value, exps):
        if value == 'MoveToPosition':
            a = self.Msg()
            a.x = exps[0]['x']
            a.y = exps[0]['y']
            self.move_to_pos(a)

    class Msg(object):
        pass

    def set_up_broadcaster(self):
        rospy.Subscriber('/%s/pose' % self.turtle_name,
                         turtlesim.msg.Pose,
                         self.handle_turtle_pose)

    def handle_turtle_pose(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent
        t.child_frame_id = self.turtle_name
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    tfb = DynamicComponent()
    rospy.sleep(2)
    program = '''
        position := {"x": 1, "y": 2};
        m_MoveToPosition(position);
        if pose.x - position.x > 0.01 || pose.y - position.y > 0.01 then
        {
            skip
        } 
        else 
        {
            skip
        };
        angles := {"yaw": 0, "pitch":3.14159/4, "roll":0};
        send(id_arm, msg_Rotate, angles);
        receive(m_Idle){(msg_MoveToPosition, position, { m_MoveToPosition(position) })}            
    '''
    tfb.execute(program)
    rospy.spin()
