#!/usr/bin/python3

import serial
import time
import re

# https://stackoverflow.com/questions/59957089/how-to-send-one-gcode-command-over-usb
# https://stackoverflow.com/questions/676172/full-examples-of-using-pyserial-package

class Crane():

    def __init__(self, port = "/dev/ttyUSB0", baud = 250000, t = 22): #115200
        self.t = t
        self.chan = serial.Serial(port, baud)
        self.receive()
        # TODO read?
        self.processCommand("G90")

    def __del__(self):
        self.chan.close()

    def send(self, command):
        self.chan.write(command.encode())
        self.chan.write(b'\r\n')

    def receive(self):
        time.sleep(0.1)
        out = ''
        while self.chan.inWaiting() > 0:
            out += self.chan.readline().decode()
        print(out)
        return out
        # TODO parse (remove last ok?)

    def processCommand(self, command):
        self.send(command)
        return self.receive()

    def home(self, x0 = None, y0 = None, z0 = None):
        self.processCommand(b"G28")

    def moveTo(self, x, y, z, x0 = None, y0 = None, z0 = None):
        if x0 != None:
            assert y0 != None
            assert z0 != None
            x = x0
            y = y0
            z = z0
        assert x >= 0 and x <= 180
        assert y >= 0 and y <= 160
        assert z >= 0 and z <= 200
        self.processCommand("G1 X" + str(x) + " Y" + str(y) + " Z" + str(z))

    def moveToXY(self, x, y, x0 = None, y0 = None):
        if x0 != None:
            assert y0 != None
            x = x0
            y = y0
        assert x >= 0 and x <= 180
        assert y >= 0 and y <= 160
        self.processCommand("G1 X" + str(x) + " Y" + str(y))

    def moveToX(self, x, x0 = None):
        if x0 != None:
            x = x0
        assert x >= 0 and x <= 180
        self.processCommand("G1 X" + str(x))

    def moveToY(self, y, y0 = None):
        if y0 != None:
            y = y0
        assert y >= 0 and y <= 160
        self.processCommand("G1 Y" + str(y))

    def moveToZ(self, z, z0 = None):
        if z0 != None:
            z = z0
        assert z >= 0 and z <= 200
        self.processCommand("G1 Z" + str(z))

    def closeGripper(self):
        self.processCommand("M106")
        time.sleep(1.0)

    def openGripper(self):
        self.processCommand("M107")
        time.sleep(1.0)

    def hasObject(self):
        result = self.processCommand("M105")
        m = re.search('T0:(\d\d.\d)', result) #TODO as comp from ambient (bed) temp
        t = float(m.group(1))
        return t <= self.t

    def grip(self):
        self.closeGripper()
        return self.hasObject()

    def idle(self):
        time.sleep(0.1)

    def wait(self, t):
        time.sleep(t)

    def moveObject(self, srcX, srcY, srcZ, fstX, dstY, dstZ):
        self.openGripper()
        self.moveToXY(srcX, srcY)
        self.moveToZ(srcZ)
        if not self.grip():
            raise Exception("grip")
        self.moveToZ(0)
        self.moveToXY(dstX, dstY)
        self.moveToZ(dstZ)
        self.openGripper()
        self.moveToZ(0)
        self.moveToXY(90, 110)

    def inverse(self, mpName, arg, error = None):
        if mpName == "moveTo":
            if len(arg) == 6:
                return mpName, arg[3:6] + arg[0:3]
            else:
                raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "moveToX" or mpName == "moveToY" or mpName == "moveToZ":
            if len(arg) == 2:
                return mpName, [arg[1], arg[0]]
            else:
                raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "moveToXY":
            if len(arg) == t:
                return mpName, arg[2:4] + arg[0:2]
            else:
                raise ValueError('cannot invert absolute motion without the pre state', mpName)
        elif mpName == "moveObject":
            assert len(arg) == 6
            if error == None:
                return mpName, arg[3:6] + arg[0:3]
            else:
                return "home", []
        elif mpName == "closeGripper":
            return "openGripper", arg
        elif mpName == "openGripper":
            return "closeGripper", arg
        elif mpName == "idle" or mpName == "wait":
            return mpName, arg
        else:
            raise ValueError('unkown motion primitive', mpName)

if __name__ == "__main__":
    c = Crane()
    c.openGripper()
    time.sleep(1.0)
    if c.grip():
        print("got object")
    else:
        print("did not get object")
    c.openGripper()
