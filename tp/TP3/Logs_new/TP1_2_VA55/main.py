#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,                            InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import sys
from RobotController import RobotController
from DistanceSensor import DistanceSensor
from ColorSensor import ColorSensor
from DisplayLcd import DisplayLcd
from RobotState import RobotState
import time
from RobotState import RobotState
from logger import Logger
from Gyrosensor import Gyrosensor



# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

ev3.speaker.beep()

robot_controller = RobotController()

distance_sensor = DistanceSensor()
color_sensor = ColorSensor()

robot_state = RobotState()
gyrosensor = Gyrosensor()

logger=Logger()

mid_val = 50


max_time = 15
delta = 10

start = time.time()

k_p = 1.3
tau = 0.1
k_i = 1
k_d = 0.10

sum_e = 0
e_t = 0

t = 0
while(True):

    speed = 220

    x_c = mid_val
    x_t = color_sensor.get_reflection()

    e_t_minus_1 = e_t  
    e_t = x_t - x_c

    sum_e += e_t

    delta = k_p*e_t + k_i*tau*sum_e + k_d/tau*(e_t-e_t_minus_1)

    if distance_sensor.get_distance() < 50:
        delta = 0
        speed = 0
        sum_e = 0

    robot_controller.drive_base.drive(speed,delta)
    giroscope = gyrosensor.get_GiroAngle()

    logger.log(t, e_t, delta, robot_state.update_state(), giroscope)

    t += 0.1
    time.sleep(tau)