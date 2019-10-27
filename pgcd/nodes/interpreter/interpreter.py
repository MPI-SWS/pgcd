import inspect
import threading
import _thread

import pgcd.msg
import rclpy
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

class Termination(Exception):
    def __init__(self, value):
        self.value = value


class Interpreter:

    def __init__(self, component_id):
        self.id = component_id
        self.parser = Parser()
        self.variables = {}
        # messages
        self.msg_types = {}
        self.send_to = {}
        self.receive_from = {}
        #
        self.subs = {}
        # TODO automatically convert between normal and Stamped messages
        for name, obj in inspect.getmembers(pgcd.msg):
            if inspect.isclass(obj):
                self.msg_types[obj.__name__] = obj

    def execute(self, code):
        try:
            program = self.parser.parse(code)
            return self.visit(program)
        except Termination as t:
            return t.value

    # receiver name and message type name
    def get_send_info(self):
        infos = []
        def visit(self, node):
            if node.tip == Type.statement:
                for stmt in node.children:
                    visit(stmt)
            elif node.tip == Type.skip:
                pass
            elif node.tip == Type.send:
                infos.append((node.comp, node.msg_types))
            elif node.tip == Type.receive:
                for a in node.actions:
                    visit(a.program)
            elif node.tip == Type._if:
                for if_stmt in node.if_list:
                    visit(if_stmt.program)
            elif node.tip == Type._print:
                pass
            elif node.tip == Type._while:
                visit(node.program)
            elif node.tip == Type.assign:
                pass
            elif node.tip == Type.motion:
                pass
            elif node.tip == Type.exit:
                pass
            else
                assert False, "no visitor for " + node.tip
        return infos
        assert False

    # sender name and message type name
    def get_receive_info(self):
        infos = []
        def visit(self, node):
            if node.tip == Type.statement:
                for stmt in node.children:
                    visit(stmt)
            elif node.tip == Type.skip:
                pass
            elif node.tip == Type.send:
                pass
            elif node.tip == Type.receive:
                for a in node.actions:
                    infos.append((node.sender, a.str_msg_type))
                    visit(a.program)
            elif node.tip == Type._if:
                for if_stmt in node.if_list:
                    visit(if_stmt.program)
            elif node.tip == Type._print:
                pass
            elif node.tip == Type._while:
                visit(node.program)
            elif node.tip == Type.assign:
                pass
            elif node.tip == Type.motion:
                pass
            elif node.tip == Type.exit:
                pass
            else
                assert False, "no visitor for " + node.tip
        return infos

    #TODO that is slow. Instead we should compile the sympy expr (https://docs.sympy.org/latest/modules/utilities/lambdify.html) and the use that function
    def calculate_sympy_exp(self, sympy_exp):
        subs = {}
        for fs in sympy_exp.free_symbols:
            try:
                subs[fs] = self.__getattribute__(str(fs))
            except AttributeError:
                subs[fs] = self.robot.__getattribute__(str(fs))
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
        elif node.tip == Type.motion:
            self.visit_motion(node)
        elif node.tip == Type.exit:
            self.visit_exit(node)
        else
            assert False, "no visitor for " + node.tip

    def visit_statement(self, node):
        # print('\n'.join(str(c) for c in node.children))
        for stmt in node.children:
            self.visit(stmt)

    #TODO fix this: proper handling of the stamped thing
    # look in geometry_msg, sensor_msg, tf2_msg for stamped version
    # otherwise look for message with the right type
    def resolve_message_type(self, msg):
        return self.msg_types[msg], True

    def visit_send(self, node):
        component = node.comp
        msg_type, stamped = self.resolve_message_type(node.msg_type)
        values = [self.calculate_sympy_exp(val) for val in node.args]
        message = msg_type()
        # fill the fields
        if stamped:
            message.header.frame_id = self.id
            message.header.stamp = rclpy.Time.now()
            for name, val in zip(message.__slots__[1].__slots__, values):
                setattr(message.__slots__[1], name, val)
        else
            for name, val in zip(message.__slots__, values):
                setattr(message, name, val)
        #
        pub = self.send_to[component][msg_type.__name__]
        pub.publish(message)
        print('sent> ' + component)
        ack = self.receive_from[component].get()
        assert ack._type == String, "not an ack!?!"

    def visit_receive(self, node):
        actions = [self.visit(a) for a in node.actions]
        next_prog = Skip()
        waiting_msg = True
        while waiting_msg:
            try:
                msg = self.receive_from[node.sender].get_nowait()
                waiting_msg = False
                for action in actions:
                    if action['msg_type'] == msg._type:
                        for name, var in zip(msg.__slots__, action['data_name']):
                            print("\nSetting:",  var, "to", name, getattr(msg, name))
                            self.__setattr__(var, getattr(msg, name))
                        next_prog = action['program']
                        break
                    # was transformed in component
                    elif action['msg_type'] + 'Stamped' == msg._type:
                        for name, var in zip(msg.__slots__[1], action['data_name']):
                            print("\nSetting:",  var, "to", name, getattr(msg, name))
                            self.__setattr__(var, getattr(msg, name))
                        next_prog = action['program']
                        break
                else:
                    assert False, "did not find handler for " + str(msg._type)
            except queue.Empty:
                self.visit_motion(node.motion)
         self.visit(next_prog)

    def transform_slots(self, msg, args):
        # print("-----> 1")
        a = time.time()
        source_pt = np.ones([4])
        source_pt[0] = msg.x
        source_pt[1] = msg.y
        source_pt[2] = msg.z
        source_pt[3] = 1
        tfBuffer = tf2_ros.Buffer()
        # print("-----> 2")
        listener = tf2_ros.TransformListener(tfBuffer)
        # print("-----> 3")
        print("Waiting transformation between: ", getattr(msg, 'source_frame'), self.id, end='')
        while True:
            #print('transform_slots transform')
            try:

                trans = tfBuffer.lookup_transform(getattr(msg, 'source_frame'), self.id, rclpy.Time())
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
                    print("\nSetting:", args['data_name'][k], end_pt[k])
                    self.__setattr__(args['data_name'][k], end_pt[k])
                    k += 1
                break
                # print("-----> 5")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # print(".", end='')
                continue
        print("Transform took seconds: " + str(time.time() - a))

    def visit_action(self, node):
        return {'msg_type': self.msg_types[node.str_msg_type], 'data_name': node.data_names, 'program': node.program}

    def visit_if(self, node):
        for if_stmt in node.if_list:
            val = self.calculate_sympy_exp(if_stmt.condition)
            if val:
                self.visit(if_stmt.program)
                break

    def visit_while(self, node):
        val = self.calculate_sympy_exp(node.condition)
        while val:
            self.visit(node.program)
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
        #rclpy.sleep( 10 ) 

    def visit_print(self, node):
        if isinstance(node.arg, str):
            print("Output:", str(node.arg))
        else:
            print("Output:", ','.join(str(self.calculate_sympy_exp(x)) for x in node.arg))
    
    def visit_exit(self, node):
        res = self.calculate_sympy_exp(node.expr)
        raise Termination(res)
