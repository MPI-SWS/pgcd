from interpreter.ast_inter import *

def contains(program, checkpoint):
    program.exists(lambda n: n.isCheckpoint() and n.ids.contains(checkpoint))

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
                        continue
            elif current.isIf():
                for i in current.if_list:
                    if contains(i, checkpoint):
                        current = i.program
                        continue
            elif current.isWhile():
                tail = [current] + tail
                current = current.program
            elif current.isReceive():
                for i in current.actions:
                    if contains(i, checkpoint):
                        current = i.program
                        continue
            else:
                assert current.isCheckpoint() and current.ids.contains(checkpoint)
                break
        return Statement([current] + tail)
    else:
        return program
