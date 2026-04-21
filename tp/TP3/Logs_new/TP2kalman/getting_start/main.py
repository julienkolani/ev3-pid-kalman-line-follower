#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import sys
import time


from control import Control
from distance import Distance
from color import Color
from LCD import LCD
from log import Logger
from sound_light import SoundLight
from PID import PID
from state import State
from pose import Pose
from Kalman import KalmanAngle


size_wheel, space_wheel = 55, 108

robot = Control(size_wheel, space_wheel)
color_sensor = Color()
distance_sensor = Distance()
lcd = LCD()
logger = Logger()
sound_light = SoundLight()


config = 1

if config == 1: # straight and curvy lines
    white, black, Kp, Ki, Kd, delta, max_delta, dt, speed, error = 59, 8, 1.75, 0.1, 0.03, 30, 180, 0.1, 150, 15
    # For smoother motion:
    # white, black, Kp, Ki, Kd, delta, max_delta, dt, speed = 59, 8, 1.4, 0.00, 0.03, 30, 120, 0.05, 250, 50

elif config == 2: # for the 8
    white, black, Kp, Ki, Kd, delta, max_delta, dt, speed, error = 59, 8, 1.55, 0.65, 0.01, 30, 180, 0.1, 260, 80
    # If you want to push speed limits
    #white, black, Kp, Ki, Kd, delta, max_delta, dt, speed = 59, 8, 1.8, 0.03, 0.12, 30, 90, 0.05, 320, 100

pid = PID(color_sensor, logger, white, black, Kp, Ki, Kd, delta, max_delta, dt)

pose = Pose(speed)

state = State(color_sensor, distance_sensor, lcd, logger, robot, sound_light, pid, speed, pose)

# Boucle principale
elapsed_time = 0.1
debut = 0
real_start = time.time()
try:
    while True:
        start_time = time.time()  # Temps de début de la boucle

        # Exécution des opérations
        state.update(elapsed_time)
        state.react("PID")  # mode PID, sans Kalman
        speed, distance, color = state.get_state_dict()
        # logger.log(debut, speed, distance, color, **pid.get_PID_dict())
        
        t = time.time() - real_start

        data = {
            "time": t,
            "pid": (pose.x_pid, pose.y_pid),
            "state": (pose.x_state, pose.y_state),
            "gyro": (pose.x_gyro, pose.y_gyro),
            "raw": {
                "drive_speed": pose.drive_speed,
                "turn_rate": pose.turn_rate,
                "state_distance": pose.state_distance,
                "state_speed": pose.state_speed,
                "state_angle": pose.state_angle,
                "gyro_angle": pose.gyro.angle()
            }
        }
        print("Data {}".format(data))
        logger.log_kalman(pose.x_pid, pose.y_pid, pose.x_state, pose.y_state, pose.x_gyro, pose.y_gyro, pose.state_distance, pose.state_angle)
        pose.history.append(data)

        # Temps écoulé depuis le début de la boucle
        elapsed_time = time.time() - start_time
         # Temps d'attente restant pour atteindre 50 ms
        sleep_time = max(0, 0.1 - elapsed_time)  # 0.1 seconde = 100 ms

        # Attente si nécessaire
        if sleep_time > 0:
           time.sleep(sleep_time)
        debut += elapsed_time

        

except KeyboardInterrupt:
    robot.stop()
    lcd.display("Arrêt manuel")
    sound_light.set_color("ORANGE")
