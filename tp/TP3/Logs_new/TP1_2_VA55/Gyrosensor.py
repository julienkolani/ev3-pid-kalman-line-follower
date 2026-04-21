#!/usr/bin/env pybricks-micropython
from  pybricks.hubs import EV3Brick
from  pybricks.ev3devices import ColorSensor as color
from pybricks.parameters import Port
from pybricks.ev3devices import GyroSensor
 
class Gyrosensor:
    def __init__(self):
        self.get_PortGiro = GyroSensor(Port.S1)
        self.port = Port.S1
        self.positive_direction = True
        self.get_PortGiro.reset_angle(0)
 
    def get_GiroAngle(self):
        return self.get_PortGiro.angle()
 