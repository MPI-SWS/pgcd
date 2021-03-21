
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
