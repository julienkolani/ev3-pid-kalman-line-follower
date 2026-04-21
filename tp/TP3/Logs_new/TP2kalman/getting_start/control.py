#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase
import math

class Control:
    def __init__(self, size_wheel=54, space_wheel=120):
        self.left_motor = Motor(Port.B)      
        self.right_motor = Motor(Port.C)
        self.robot = DriveBase(self.left_motor, self.right_motor, size_wheel, space_wheel)

    def forward(self, speed):
        # self.left_motor.run(speed)
        # self.right_motor.run(speed)
        self.robot.drive(speed, 0)

    def curve(self, aSpeed, angle):
        # self.left_motor.run_angle(aSpeed, -angle)
        # self.right_motor.run_angle(aSpeed, angle)
        self.robot.drive(aSpeed, angle)

    def stop(self):
        # self.left_motor.stop()
        # self.right_motor.stop()
        self.robot.stop()

    def get_state(self):
        return self.robot.state()