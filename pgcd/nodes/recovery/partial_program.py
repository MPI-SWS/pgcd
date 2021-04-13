from interpreter.ast_inter import *

def resumeAt(program, checkpoint):
    assert checkpoint.isCheckpoint()
    assert program.contains(checkpoint)
    tail = []
    current = program
    while True:
        if current.isBlock():
            for i in range(0, len(current.children)):
                if current.children[i].contains(checkpoint):
                    tail = current.children[i+1:] + tail
                    current = current.children[i]
                    break
        elif current.isIf():
            for i in current.if_list:
                if i.contains(checkpoint):
                    current = i.program
                    break
        elif current.isWhile():
            tail = [current] + tail
            current = current.program
        elif current.isReceive():
            for i in current.actions:
                if i.contains(checkpoint):
                    current = i.program
                    break
            pass
        else:
            assert current == checkpoint
            return Statement([checkpoint] + tail)
