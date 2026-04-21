#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks import ev3brick as brick

class State:
    def __init__(self, color_sensor, distance_sensor, lcd, logger, motor, sound_light, pid, speed, pose):
        self.color_sensor = color_sensor
        self.distance_sensor = distance_sensor
        self.lcd = lcd
        self.logger = logger
        self.motor = motor
        self.sound_light = sound_light
        self.pid = pid
        self.pose = pose

        self.color = None
        self.distance = 0
        self.speed = speed


    def update(self, elapsed_time):
        # self.color = self.color_sensor.color()
        # self.pid.dt = elapsed_time
        self.distance = self.distance_sensor.get_distance()
        print("State : {}".format(self.motor.get_state()))
        # self.lcd.display("Color: {}, Distance: {}".format(self.color, self.distance))


    def react(self, mode="bangbang"):
        if mode == "bangbang":
            angle = self.pid.bangbang_angle()
        elif mode == "P":  # mode proportionnel
            angle = self.pid.calc_P()
        elif mode == "PI":
            angle = self.pid.calc_PI()
        elif mode == "PID":
            angle = self.pid.calc_PID()
        else:
            angle = 0
            
        if self.distance <= 150:
            self.motor.stop()
        else:
            
            self.motor.curve(self.speed, angle)
            self.pose.update(angle, self.motor.get_state())

        


    def get_state_dict(self):
        return {
            self.speed,
            self.distance,
            self.color,
        }

        
