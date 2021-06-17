#!/usr/bin/python3

import serial
from time import sleep, time
import re

# https://stackoverflow.com/questions/59957089/how-to-send-one-gcode-command-over-usb
# https://stackoverflow.com/questions/676172/full-examples-of-using-pyserial-package

class Crane():

    def __init__(self, port = "/dev/ttyUSB0", baud = 115200)
        self.chan = serial.Serial(port, baud)

    def send(self, command):
        self.chan.write(command)
        self.chan.write("\r\n")
        #TODO set mode for switch pin

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
        self.processCommand("M106 S245")
    
    def openGripper(self):
        self.processCommand("M106 S10")

    def hasObject(self):
        # TODO check is the gripper switch has something (M43 P<pin> ?)
        result = self.processCommand("M43 P???")
        pin = int(re.find('Input = \d').split[-1])
        return pin == 1

    def grip(self):
        self.closeGripper()
        return self.hasObject()
    
    def idle(self):
        time.sleep(0.1)

    def moveObject(self, srcX, srcY, srcZ, fstX, dstY, dstZ):
        self.moveToZ(0)
        self.moveToXY(srcX, srcY)
        self.moveToZ(srcZ)
        if not self.grip():
            raise Exception("grip")
        self.moveToZ(0)
        self.moveToXY(dstX, dstY)
        self.moveToZ(dstZ)
        sefl.openGripper()
        self.moveToZ(0)

