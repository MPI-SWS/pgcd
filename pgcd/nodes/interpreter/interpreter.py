import inspect
#import threading
#import _thread

import pgcd.msg
import rclpy
from rclpy.logging import LoggingSeverity
import tf2_ros
import geometry_msgs.msg
import std_msgs.msg

import queue
import numpy as np
from sympy.core.numbers import Float, Zero, One, NegativeOne

from interpreter.parser import *

class Termination(Exception):
    def __init__(self, value):
        self.value = value


class Interpreter:

    def __init__(self, component_id):
        self.id = component_id
        self.parser = Parser()
        self.variables = {}
        self.program = Skip()
        # messages
        self.msg_types = {}
        self.send_to = {}
        self.receive_from = {}
        # automatically convert between normal and Stamped messages
        def addMsg(obj):
            if inspect.isclass(obj):
                suffix = 'Stamped'
                if obj.__name__.endswith(suffix):
                    self.msg_types[obj.__name__[:-len(suffix)]] = obj
                elif obj.__name__ not in self.msg_types:
                    self.msg_types[obj.__name__] = obj
        for name, obj in inspect.getmembers(pgcd.msg): addMsg(obj)
        for name, obj in inspect.getmembers(geometry_msgs.msg): addMsg(obj)
        for name, obj in inspect.getmembers(std_msgs.msg): addMsg(obj)

    def execute(self):
        try:
            return self.visit(self.program)
        except Termination as t:
            return t.value

    def parse(self, code):
        self.program = self.parser.parse(code)

    # receiver name and message type name
    def get_send_info(self):
        infos = []
        def visit(node):
            if node.tip == Type.statement:
                for stmt in node.children:
                    visit(stmt)
            elif node.tip == Type.skip:
                pass
            elif node.tip == Type.send:
                infos.append((node.comp, node.msg_type))
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
            else:
                assert False, "no visitor for " + node.tip
        visit(self.program)
        return infos

    # sender name and message type name
    def get_receive_info(self):
        infos = []
        def visit(node):
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
            else:
                assert False, "no visitor for " + node.tip
        visit(self.program)
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
        if expr2 == S.true:
            return True
        elif expr2 == S.false:
            return False
        else:
            evaluated = N(expr2)
            if type(evaluated) == Float:
                return float(evaluated)
            elif type(evaluated) == Zero or type(evaluated) == One or type(evaluated) == NegativeOne:
                return float(evaluated)
                #return int(evaluated)
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
        else:
            assert False, "no visitor for " + node.tip

    def visit_statement(self, node):
        # print('\n'.join(str(c) for c in node.children))
        for stmt in node.children:
            self.visit(stmt)

    def resolve_message_type(self, msg):
        obj = self.msg_types[msg]
        stamped = obj.__slots__[0] == '_header'
        return obj, stamped

    def visit_send(self, node):
        component = node.comp
        msg_type, stamped = self.resolve_message_type(node.msg_type)
        values = [self.calculate_sympy_exp(val) for val in node.args]
        message = msg_type()
        # fill the fields
        if stamped:
            message.header.frame_id = self.id
            #message.header.stamp = rclpy.Time.now() #FIXME
            unstamped = getattr(message, message.__slots__[1])
            for name, val in zip(unstamped.__slots__, values):
                #print(name, val, type(val))
                setattr(unstamped, name, val)
        else:
            for name, val in zip(message.__slots__, values):
                setattr(message, name, val)
        #
        pub = self.send_to[component][node.msg_type]
        pub.publish(message)
        rclpy.logging._root_logger.log("sent to " + component + " " + str(message), LoggingSeverity.INFO)
        ack = self.receive_from[component].get()
        assert type(ack) == std_msgs.msg.String, "not an ack!?!"
        assert ack.data == component, "not an ack!?!"

    def visit_receive(self, node):
        actions = [self.visit(a) for a in node.actions]
        next_prog = Skip()
        waiting_msg = True
        while waiting_msg:
            try:
                msg = self.receive_from[node.sender].get_nowait()
                ack = std_msgs.msg.String()
                ack.data = self.id
                self.send_to[node.sender]["Ack"].publish(ack)
                waiting_msg = False
                for action in actions:
                    if action['msg_type'] == type(msg):
                        if action['msg_type'].__name__.endswith("Stamped"):
                            inner_msg = getattr(msg, msg.__slots__[1]) #assume header 1st
                            for name, var in zip(inner_msg.__slots__, action['data_name']):
                                val = getattr(inner_msg, name)
                                rclpy.logging._root_logger.log("Setting: " + var + " to " + name + " = " + str(val), LoggingSeverity.DEBUG)
                                self.__setattr__(var, val)
                            next_prog = action['program']
                        else:
                            for name, var in zip(msg.__slots__, action['data_name']):
                                val = getattr(msg, name)
                                rclpy.logging._root_logger.log("Setting: " + var + " to " + name + " = " + str(val), LoggingSeverity.DEBUG)
                                self.__setattr__(var, val)
                            next_prog = action['program']
                        break
                else:
                    assert False, "did not find handler for " + str(msg)
            except queue.Empty:
                self.visit_motion(node.motion)
        self.visit(next_prog)

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
        if not hasattr(self.robot, node.value):
            rclpy.logging._root_logger.log("visit_motion: could not find " + node.value, LoggingSeverity.WARNING)
            pass
        else:
            if node.value != "idle":
                rclpy.logging._root_logger.log("visit_motion: " + str(node), LoggingSeverity.INFO)
            try:
                getattr(self.robot, node.value)(*list(self.calculate_sympy_exp(x) for x in node.exps))
            except Exception as e:
                rclpy.logging._root_logger.log("visit_motion generated and error: " + str(e), LoggingSeverity.ERROR)

    def visit_print(self, node):
        if isinstance(node.arg, str):
            rclpy.logging._root_logger.log("print: " + str(node.arg), LoggingSeverity.INFO)
        else:
            rclpy.logging._root_logger.log("print: " + ','.join(str(self.calculate_sympy_exp(x)) for x in node.arg), LoggingSeverity.INFO)
    
    def visit_exit(self, node):
        res = self.calculate_sympy_exp(node.expr)
        raise Termination(res)
