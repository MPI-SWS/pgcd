from interpreter.ast_inter import *

# receiver name and message type name
def get_send_info(program):
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
        elif node.tip == Type.checkpoint:
            pass
        else:
            assert False, "no visitor for " + node.tip
    visit(program)
    return infos

# sender name and message type name
def get_receive_info(program):
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
        elif node.tip == Type.checkpoint:
            pass
        else:
            assert False, "no visitor for " + node.tip
    visit(program)
    return infos
