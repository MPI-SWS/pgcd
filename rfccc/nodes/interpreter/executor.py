import inspect
import threading
import _thread

import rfccc.msg
import rospy
import tf2_ros
import tf2_py
import geometry_msgs.msg as geom
import tf2_geometry_msgs
from std_msgs.msg import Header
import tf
import time

import numpy as np
import tf2_ros.buffer_interface

from parser import *


class Executor:

    def __init__(self, component_id):
        self.id = component_id
        self.parser = Parser()
        self.waiting_msg = False
        self.variables = {}
        self.msg_types = {}
        self.subs = {}
        self.lock = threading.Lock()
        for name, obj in inspect.getmembers(rfccc.msg):
            if inspect.isclass(obj):
                self.msg_types[obj.__name__] = obj

    def execute(self, code):
        self.parser.parse(code).accept(self)

    def calculate_sympy_exp(self, sympy_exp):
        subs = {}
        for fs in sympy_exp.free_symbols:
            subs[fs] = self.__getattribute__(str(fs))
        expr2 = sympy_exp.subs(subs)
        if expr2 == S.true or expr2 == S.false:
            return expr2
        else:
            evaluated = N(expr2)
            return evaluated

    def visit(self, node):

        if node.tip == Type.statement:
            self.visit_statement(node)

        elif node.tip == Type.skip:
            pass

        elif node.tip == Type.send:
            self.visit_send(node)

        elif node.tip == Type.receive:
            self.visit_receive(node)

        elif node.tip == Type.action:
            return self.visit_action(node)

        elif node.tip == Type._if:
            self.visit_if(node)

        elif node.tip == Type._print:
            self.visit_print(node)

        elif node.tip == Type._while:
            self.visit_while(node)

        elif node.tip == Type.assign:
            self.visit_assign(node)

        elif isinstance(node, Motion):
            self.visit_motion(node)

    def visit_statement(self, node):
        # print('\n'.join(str(c) for c in node.children))
        for stmt in node.children:
            stmt.accept(self)

    def visit_send(self, node):
        component, msg_type = node.comp, self.msg_types[node.msg_type]
        values = [self.calculate_sympy_exp(val) for val in node.args]
        message = msg_type()
        if 'source_frame' in message.__slots__:
            values.insert(message.__slots__.index('source_frame'), self.id)
        for name, val in zip(message.__slots__, values):
            setattr(message, name, val)

        # print("/" + component, )
        self.pub = rospy.Publisher("/" + component, msg_type, queue_size=1)
        print(self.id + ' ', end='')
        while self.pub.get_num_connections() == 0:
            #print('visit_send wait connect')
            pass
        print('sent> ' + component)
        while self.pub.get_num_connections() != 0:
            #print('visit_send wait disconnect')
            self.pub.publish(message)
        print('stop sending> ' + self.id)
        self.pub.unregister()

    def visit_receive(self, node):
        actions = [a.accept(self) for a in node.actions]
        self.waiting_msg = True
        for action in actions:
            # print("/" + self.id)
            self.subs["/" + self.id + '/' + action['msg_type'].__name__] = \
                rospy.Subscriber("/" + self.id, action['msg_type'],
                                 self.process_msg,
                                 callback_args=action,
                                 queue_size=1)
        while self.waiting_msg:
            #print('visit_rceive wait')
            self.visit_motion(node.motion)

    def process_msg(self, msg, args):
        if self.waiting_msg:
            print('stop receiving> ' + self.id)
            for key in self.subs.keys():
                self.subs[key].unregister()

            if 'source_frame' in msg.__slots__:
                self.transform_slots(msg, args)
            else:
                for name in msg.__slots__:
                    print("\nSetting:", args['data_name'] + '_' + name, getattr(msg, name))
                    self.__setattr__(args['data_name'] + '_' + name, getattr(msg, name))
            args['program'].accept(self)
            self.waiting_msg = False


    def transform_slots(self, msg, args):
        # print("-----> 1")
        a = time.time()
        source_pt = np.ones([4])
        source_pt[0], source_pt[1], source_pt[2], source_pt[3] = getattr(msg, 'x'), getattr(msg, 'y'), getattr(msg,
                                                                                                               'z'), 1
        tfBuffer = tf2_ros.Buffer()
        # print("-----> 2")
        listener = tf2_ros.TransformListener(tfBuffer)
        # print("-----> 3")
        print("Waiting transformation between: ", getattr(msg, 'source_frame'), self.id, end='')
        while True:
            #print('transform_slots transform')
            try:

                trans = tfBuffer.lookup_transform(getattr(msg, 'source_frame'), self.id, rospy.Time())
                x = tf.transformations.quaternion_matrix(np.array([trans.transform.rotation.x,
                                                                   trans.transform.rotation.y,
                                                                   trans.transform.rotation.z,
                                                                   trans.transform.rotation.w]))
                # print("-----> 3.6")
                x[0, 3] = trans.transform.translation.x
                x[1, 3] = trans.transform.translation.y
                x[2, 3] = trans.transform.translation.z
                end_pt = np.dot(x, source_pt)
                # print(x)
                # print(source_pt)
                # print(end_pt)
                k = 0
                for name in msg.__slots__:
                    if name == 'source_frame':
                        continue
                    print("\nSetting:", args['data_name'] + '_' + name, end_pt[k])
                    self.__setattr__(args['data_name'] + '_' + name, end_pt[k])
                    k += 1
                break
                # print("-----> 5")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # print(".", end='')
                continue
        print("Transform took seconds: " + str(time.time() - a))

    def visit_action(self, node):
        return {'msg_type': self.msg_types[node.str_msg_type], 'data_name': node.data_name, 'program': node.program}

    def visit_if(self, node):
        for if_stmt in node.if_list:
            val = self.calculate_sympy_exp(if_stmt.condition)
            if val:
                if_stmt.program.accept(self)
                break

    def visit_while(self, node):
        val = self.calculate_sympy_exp(node.condition)
        while val:
            node.program.accept(self)
            val = self.calculate_sympy_exp(node.condition)
            #print('visit_while transform')

    def visit_assign(self, node):
        self.__setattr__(node.id, self.calculate_sympy_exp(node.value))

    def visit_motion(self, node):
        if not hasattr(self.robot, node.value): pass
        print( "visit_motion node:",node )
        try:
            print( "visit_motion", node.value, list( (self.calculate_sympy_exp(x) for x in node.exps) ) )

            #_thread.start_new_thread( getattr(self.robot, node.value), (*list(self.calculate_sympy_exp(x) for x in node.exps), ))


            getattr(self.robot, node.value)(*list(self.calculate_sympy_exp(x) for x in node.exps))
            print( "success" )
        except Exception as e:
            pass
            print( "visit_motion generated error", str(e) )
        #rospy.sleep( 10 ) 

    def visit_print(self, node):
        if isinstance(node.arg, str):
            print("Output:", str(node.arg))
        else:
            print("Output:", ','.join(str(self.calculate_sympy_exp(x)) for x in node.arg))
