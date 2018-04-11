#!/usr/bin/env python3

import rospy
import _thread
import tf_updater
from executor import Executor
import cartandarm

class Component(Executor):

    def __init__(self):
        Executor.__init__(self, rospy.get_name())
        self.id = rospy.get_name()[1:]
        self.components = {}
        #self.prog_path = rospy.get_param('~program_location') + self.id + '.rosl'
        self.prog_path = '/home/pi/ros_catkin_ws/src/rfccc/nodes/programs/' + self.id + '.rosl'
        print( self.prog_path )

        #try:
        #    self.parent = rospy.get_param('~parent')
        #except KeyError as e:
        #    print(str(e))
        #    self.parent = "world"
        ##print(self.parent)

        #try:
        #    self.x = rospy.get_param('~x')
        #    self.y = rospy.get_param('~y')
        #    self.z = rospy.get_param('~z')
        #except KeyError as e:
        #    print(str(e))
        #    self.x = 0
        #    self.y = 0
        #    self.z = 0

        # self.robot = en.robot_arm(self.x, self.y, self.z)
        # self.robot.set_angle_base(0, 0, 0)
        # self.robot.set_angle_elbow(0, 0, 0)
        # self.robot.calculateEndPosition()
        #self.init_parts()
        

        if self.id == "arm":
            self.robot = cartandarm.arm()
            tf_updater.TFUpdater( "_frame_0", "cart_frame", self.robot )
        else:
            self.robot = cartandarm.cart()
            tf_updater.TFUpdater( "cart_frame", "world", self.robot )
        
        #self.tf_updater = tf_updater.TFUpdater(self.id, self.parent)
        self.execute_prog()

    # def init_parts(self):
    #     self.components[self.robot.list[0].getName()] = tf_updater.TFUpdater(self.id, self.parent, self.robot.list[0], self.robot.list)
    #     for i in range(1, len(self.robot.list)):
    #         parent = self.id + "_" + self.robot.list[i-1].getName()
    #         self.components[self.robot.list[i].getName()] = tf_updater.TFUpdater(self.id, parent,
    #                                                                             self.robot.list[i], self.robot.list)


    
    
    def execute_prog(self):
        # try:
            with open(self.prog_path, 'r') as content_file:
                program = content_file.read()
            self.execute(program)
        # except Exception as e:
        #     print(str(e))

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = Component()
    rospy.spin()
