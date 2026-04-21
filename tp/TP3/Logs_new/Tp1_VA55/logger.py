#!/usr/bin/env pybricks-micropython
import time
import csv
from pybricks.tools import DataLog
from RobotState import RobotState


class Logger:
    def __init__(self, filename="robot_log.csv"):
        self.filename = filename
        self.data_log = DataLog("time", "error","delta", "distance", "speed","angle","turn_angle","gyro_cumulative_angle", name=filename, timestamp=True, extension='csv', append=False)

    def log(self, time, error, delta, state, gyroscope):
        self.data_log.log(time, error, delta, state[0], state[1], state[2], state[3], gyroscope)