#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port

class DisplayLcd:
    def __init__(self):
        self.ev3 = EV3Brick()

    

    def display_state(self, state):
        self.ev3.screen.clear()
        self.ev3.screen.draw_text(10, 10, "Speed: {} mm/s".format(self.ev3.drive.speed()))
        self.ev3.screen.draw_text(10, 30, "State: {}".format(state))
        self.ev3.screen.draw_text(10, 50, "Color: {}".format(self.color_sensor.get_color()))
        self.ev3.screen.draw_text(10, 70, "Distance: {} mm".format(self.distance_sensor.get_distance()))

    def display_message(self, message):
        self.ev3.screen.clear()
        self.ev3.screen.draw_text(10, 10, message)