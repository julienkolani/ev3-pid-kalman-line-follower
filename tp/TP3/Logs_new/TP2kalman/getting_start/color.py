#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color

class Color:
    def __init__(self):
        self.color_sensor = ColorSensor(Port.S3)

    def color(self):
        return self.color_sensor.color()

    def detect_floor_zone(self, target_colors):
        current_color = self.color()
        if current_color in target_colors:
            return True
        return False

    def detect_line(self):
        current_color = self.color()
        if current_color == Color.BLACK:  
            return True
        return False

    def detect_light(self):
        return self.color_sensor.reflection()