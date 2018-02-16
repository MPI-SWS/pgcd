#!/usr/bin/env python
from math import pow, atan2, sqrt

import geometry_msgs.msg
import rospy
import tf
import tf2_ros
import turtlesim.msg
import turtlesim.srv
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from executor import Executor

'''
Class currently represents ROS turtle1 in turtlesim. 
Every node has to broadcast its position in the tf2 library.
'''


class DynamicComponent(Executor):

    def __init__(self):
        Executor.__init__(self, rospy.get_param('~turtle'))
        self.id = rospy.get_param('~turtle')
        self.prog_path = rospy.get_param('~program_location') + self.id + '.rosl'
        self.parent = "world"
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel' % self.id, Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/%s/pose' % self.id, Pose, self.update_position)
        self.pose = Pose()
        self.rate = rospy.Rate(20)
        self.set_up_broadcaster()
        rospy.sleep(1)
        with open(self.prog_path, 'r') as content_file:
            program = content_file.read()
        self.execute(program)

    def update_position(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def move_to_pos(self, pos):
        if hasattr(self, 'sub'):
            self.sub.unregister()
        self.waiting_msg = False
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
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def set_up_broadcaster(self):
        rospy.Subscriber('/%s/pose' % self.id,
                         turtlesim.msg.Pose,
                         self.handle_turtle_pose)

    def handle_turtle_pose(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent
        t.child_frame_id = self.id
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
    rospy.spin()
