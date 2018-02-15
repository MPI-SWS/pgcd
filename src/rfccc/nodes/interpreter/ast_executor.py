from __future__ import print_function

import inspect
import threading

import rfccc.msg
import rospy

from parser import *


class Executor:

    def __init__(self, component_id):
        self.id = component_id
        self.parser = RoboParser()
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

    def visit_statement(self, node):
        for stmt in node.children:
            stmt.accept(self)

    def visit_send(self, node):
        component, msg_type, value = node.comp, self.msg_types[node.msg_type], node.value.accept(self)
        message = msg_type()
        for name in message.__slots__:
            setattr(message, name, value[name])

        self.pub = rospy.Publisher("/" + component, msg_type, queue_size=3)
        print(self.id + ' ', end='')
        while self.pub.get_num_connections() == 0:
            print('-', end='')
            rospy.sleep(0.1)
        print('> ' + component)
        self.pub.publish(message)
        self.pub.unregister()

    def visit_receive(self, node):
        actions = [a.accept(self) for a in node.actions]
        self.waiting_msg = True
        for action in actions:
            self.subs["/" + self.id + '/' + action['msg_type'].__name__] = \
                rospy.Subscriber("/" + getattr(self, 'id'), action['msg_type'],
                                 self.process_msg,
                                 callback_args=action,
                                 queue_size=100)
        while self.waiting_msg:
            self.visit_motion(node.motion)

    def process_msg(self, msg, args):
        with self.lock:
            if self.waiting_msg:
                for sub in self.subs:
                    sub.unregister()
                data = {}
                for name in msg.__slots__:
                    data[name] = getattr(msg, name)
                self.variables[args['data_name']] = data
                args['program'].accept(self)
                self.waiting_msg = False

    def visit_action(self, node):
        self.variables[node.data_name] = {}
        assert node.str_msg_type in self.msg_types
        return {'msg_type': self.msg_types[node.str_msg_type], 'data_name': node.data_name, 'program': node.program}

    def visit_if(self, node):
        cond = node.condition.accept(self)
        if cond:
            node.ifCode.accept(self)
        else:
            node.elseCode.accept(self)

    def visit_while(self, node):
        cond = node.condition.accept(self)
        while cond:
            node.code.accept(self)
            cond = node.condition.accept(self)

    def visit_assign(self, node):
        exp = node.value.accept(self)
        if node.property is None:
            self.variables[node.id] = exp
        else:
            self.variables[node.id][node.property] = exp

    def visit_tuple(self, node):
        dic = {}
        for key, value in node.tup.items():
            v = value.accept(self)
            dic[key] = v
        return dic

    def visit_bin_op(self, node):
        if node.tip == Type.dot:
            if node.exp1 in self.variables.keys():
                return self.variables[node.exp1][node.exp2]
            else:
                return getattr(self, node.exp1).__getattribute__(node.exp2)
        else:
            var1 = node.exp1.accept(self)
            var2 = node.exp2.accept(self)
            if isinstance(var1, str) or isinstance(var2, str):
                var1 = '"' + str(var1) + '"'
                var2 = '"' + str(var2) + '"'
            return eval(str(var1) + ' ' + str(node.sign) + ' ' + str(var2))

    def visit_un_op(self, node):
        if node.tip == Type.id:
            if node.exp in self.variables.keys():
                return self.variables[node.exp]
            else:
                return getattr(self, node.exp1)
        if node.tip == Type._print:
            for item in node.exp:
                var1 = item.accept(self)
                print(var1)
            return None
        var1 = node.exp.accept(self)
        if isinstance(var1, str):
            var1 = '"' + str(var1) + '"'
        return eval(str(node.sign) + ' (' + str(var1) + ') ')

    def visit_motion(self, node):

        class Msg(object):
            pass

        value = node.value
        exps = [x.accept(self) for x in node.exps]
        if value == 'MoveToPosition':
            a = Msg()
            a.x = exps[0]['x']
            a.y = exps[0]['y']
            self.move_to_pos(a)
        elif value == 'Rotate':
            for i in range(0, 10):
                self.yaw += exps[0].yaw / 10
                self.pitch += exps[0].pitch / 10
                self.roll += exps[0].roll / 10
                rospy.sleep(0.5)


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

        elif node.tip == Type._tuple:
            return self.visit_tuple(node)

        elif isinstance(node, BinOp):
            return self.visit_bin_op(node)

        elif isinstance(node, UnOp):
            return self.visit_un_op(node)

        elif isinstance(node, Constant):
            return node.value

        elif isinstance(node, Motion):
            self.visit_motion(node)
