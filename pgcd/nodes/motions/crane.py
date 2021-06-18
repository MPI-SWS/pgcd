#!/usr/bin/python3

import serial
from time import sleep, time
import re

# https://stackoverflow.com/questions/59957089/how-to-send-one-gcode-command-over-usb
# https://stackoverflow.com/questions/676172/full-examples-of-using-pyserial-package

class Crane():

    def __init__(self, port = "/dev/ttyUSB0", baud = 250000) #115200
        self.chan = serial.Serial(port, baud)

    def send(self, command):
        self.chan.write(command)
        self.chan.write("\r\n")

    def receive(self):
        time.sleep(0.1)
        out = ''
        while ser.inWaiting() > 0:
            out += ser.read(1)
        return out
        # TODO parse (remove last ok?)

    def processCommand(self, command):
        self.send(command)
        return self.receive()

    def home(self):
        self.processCommand("G28 Z")
        self.processCommand("G28 X")
        self.processCommand("G28 Y")

    def moveTo(self, x, y, z):
        self.processCommand("G1 X" + str(x) + " Y" + str(y) + " Z" + str(z))

    def moveToXY(self, x, y):
        self.processCommand("G1 X" + str(x) + " Y" + str(y))

    def moveToX(self, x):
        self.processCommand("G1 X" + str(x))

    def moveToY(self, y):
        self.processCommand("G1 Y" + str(y))

    def moveToZ(self, z):
        self.processCommand("G1 Z" + str(z))

    def closeGripper(self):
        self.processCommand("M106")
        #self.processCommand("M106 S245")

    def openGripper(self):
        self.processCommand("M107")
        #self.processCommand("M106 S10")

    def hasObject(self):
        result = self.processCommand("M105")
        m = re.search('T0:(\d\d.\d)', result)
        t = float(m.group(1))
        return t <= 0.0

    def grip(self):
        self.closeGripper()
        return self.hasObject()

    def idle(self):
        time.sleep(0.1)

    def moveObject(self, srcX, srcY, srcZ, fstX, dstY, dstZ):
        self.openGripper()
        self.moveToZ(0)
        self.moveToXY(srcX, srcY)
        self.moveToZ(srcZ)
        if not self.grip():
            raise Exception("grip")
        self.moveToZ(0)
        self.moveToXY(dstX, dstY)
        self.moveToZ(dstZ)
        self.openGripper()
        self.moveToZ(0)

