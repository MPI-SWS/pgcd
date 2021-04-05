
# FIXME This is a quick hack

# TODO check in interruptible and ∞ from duration
def addIdle(m):
    # m is a motion arg
    if m.mp_name == "Idle" or \
       m.mp_name == "idle" or \
       m.mp_name.endswith("_idle"):
        pass
    else:
        m.mp_name = m.mp_name + "_idle"

def addWait(m, time):
    # m is a motion arg, time is a number
    if m.mp_name == "Wait" or \
       m.mp_name == "wait":
        m.mp_args[0] = m.mp_args[0] + time #TODO wait can have 2 args
    elif m.mp_name.endswith("_wait"):
        m.mp_args[-1] = m.mp_args[-1] + time
    else:
        m.mp_name = m.mp_name + "_wait"
        m.mp_args.append(time)

def decompose(m):
    if m.mp_name == "Idle" or \
       m.mp_name == "idle" or \
       m.mp_name.endswith("_idle"):
        return (True, -1)
    elif m.mp_name == "Wait" or \
         m.mp_name == "wait" or \
         m.mp_name.endswith("_wait"):
        return (False, m.mp_args[-1])
    else:
        return (False, -1)

def decomposeExtraOnly(m):
    if m.mp_name.endswith("_idle"):
        return (m.mp_name[:-len("_idle")], True, -1)
    elif m.mp_name.endswith("_wait"):
        return (m.mp_name[:-len("_wait")], False, m.mp_args[-1])
    else:
        return (m.mp_name, False, -1)
