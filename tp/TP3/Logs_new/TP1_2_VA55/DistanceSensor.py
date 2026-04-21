#!/usr/bin/env pybricks-micropython
from  pybricks.hubs import EV3Brick
from  pybricks.ev3devices import UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

class DistanceSensor:
 
    def __init__(self):
        self.ultrasonic_sensor = UltrasonicSensor(Port.S2)
    
    def get_distance(self):
        return self.ultrasonic_sensor.distance()

    def get_driven_distance(self, drive_base):
        return drive_base.distance()