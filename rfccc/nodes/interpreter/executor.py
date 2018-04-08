import inspect
import threading

import rfccc.msg
import rospy

import math

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
            subs[fs] = self.__getattribute__(fs)
        return sympy_exp.evalf(subs=subs)

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

        elif node.tip == Type._while:
            self.visit_while(node)

        elif node.tip == Type.assign:
            self.visit_assign(node)

        elif isinstance(node, Motion):
            self.visit_motion(node)

    def visit_statement(self, node):
        for stmt in node.children:
            stmt.accept(self)

    def visit_send(self, node):
        component, msg_type = node.comp, self.msg_types[node.msg_type]
        values = [self.calculate_sympy_exp(val) for val in node.args]
        message = msg_type()
        for name, val in zip(message.__slots__, values):
            setattr(message, name, val)

        print("/" + component,)
        self.pub = rospy.Publisher("/" + component, msg_type, queue_size=3)
        print(self.id + ' ', end='')
        while self.pub.get_num_connections() == 0:
            print('-', end='')
            rospy.sleep(0.2)
        print('> ' + component)
        self.pub.publish(message)
        while self.pub.get_num_connections() != 0:
            rospy.sleep(0.05)
        self.pub.unregister()

    def visit_receive(self, node):
        actions = [a.accept(self) for a in node.actions]
        self.waiting_msg = True
        for action in actions:
            print("/" + self.id)
            self.subs["/" + self.id + '/' + action['msg_type'].__name__] = \
                rospy.Subscriber("/" + self.id, action['msg_type'],
                                 self.process_msg,
                                 callback_args=action,
                                 queue_size=100)
        while self.waiting_msg:
            self.visit_motion(node.motion)
            rospy.sleep(0.01)

    def process_msg(self, msg, args):
        with self.lock:
            if self.waiting_msg:
                for key in self.subs.keys():
                    self.subs[key].unregister()
                data = {}
                for name in msg.__slots__:
                    self.__setattr__(args['data_name'] + '_' + name, getattr(msg, name))
                args['program'].accept(self)
                self.waiting_msg = False

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
            node.code.accept(self)
            val = self.calculate_sympy_exp(node.condition)

    def visit_assign(self, node):
        self.__setattr__(node.id, node.value)

    def visit_motion(self, node):
        pass
