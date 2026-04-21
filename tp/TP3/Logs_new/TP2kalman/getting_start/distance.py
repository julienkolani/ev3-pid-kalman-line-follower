#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (InfraredSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog

class Distance:
    def __init__(self, mode="us"):
        if mode == "ir":
            self.sensor = InfraredSensor(Port.S2)
        elif mode =="us":
            self.sensor = UltrasonicSensor(Port.S2)

    def set_sensor(self, _sensor):
        self.sensor = _sensor

    def get_distance(self):
        return self.sensor.distance()