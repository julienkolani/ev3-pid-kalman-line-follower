#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick

class SoundLight:
    def light(self, color):
        brick.light(color)

    def turnoff_light(self, bool):
        brick.light(None)

    def beep(self, pitch=1000, time=500, volume=100):
        brick.sound.beep(pitch, time, volume)