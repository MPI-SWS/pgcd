from interpreter.ast_inter import *

def contains(program, checkpoint):
    return program.exists(lambda n: n.isCheckpoint() and checkpoint in n.ids)

def resumeAt(program, checkpoint):
    if checkpoint != None:
        assert contains(program, checkpoint)
        tail = []
        current = program
        while True:
            if current.isBlock():
                for i in range(0, len(current.children)):
                    if contains(current.children[i], checkpoint):
                        tail = current.children[i+1:] + tail
                        current = current.children[i]
                        break
            elif current.isIf():
                for i in current.if_list:
                    if contains(i, checkpoint):
                        current = i.program
                        break
            elif current.isWhile():
                tail = [current] + tail
                current = current.program
            elif current.isReceive():
                for i in current.actions:
                    if contains(i, checkpoint):
                        current = i.program
                        break
            else:
                assert current.isCheckpoint() and checkpoint in current.ids
                break
        return Statement([current] + tail)
    else:
        return program
