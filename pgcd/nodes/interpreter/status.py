from enum import Enum

class Termination(Exception):
    def __init__(self, value):
        self.value = value

class InterpreterStatus(Enum):
    IDLE = 1
    RUNNING = 2
    INTERRUPTED = 3
    ERROR = 4
    TERMINATED = 5

class ActionType(Enum):
    MOTION = 1
    MESSAGE = 2
    CHECKPOINT = 3
    FAILEDMOTION = 4
    MESSAGERCV = 5 # small delays can insert idle in the stack before messages, keep track of receive to flush them out
