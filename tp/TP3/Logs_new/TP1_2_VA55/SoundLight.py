#!/usr/bin/env pybricks-micropython
from  pybricks.hubs import EV3Brick
from  pybricks.ev3devices import ColorSensor as color
from pybricks.parameters import Port

class SoundLight:
    def __init__(self):
        self.ev3 = EV3Brick()
       
    def play_sound(self, frequency=1000, duration=500):
        self.ev3.speaker.beep(frequency, duration)
    
    def set_light_color(self, color):
        self.ev3.light.on(color)
    
    def turn_off_light(self):
        self.ev3.light.off()
        