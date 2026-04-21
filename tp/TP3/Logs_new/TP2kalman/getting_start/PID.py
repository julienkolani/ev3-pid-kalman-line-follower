#!/usr/bin/env pybricks-micropython
import math
from fifo import FIFO
# 2 0.03 0.06
class PID:
    def __init__(self, color_sensor, logger, white=59, black=8, Kp=2.0, Ki=0.06, Kd=0.06, delta=30, max_delta=100, dt=0.1, error=10):
        self.color_sensor = color_sensor
        self.logger = logger
        self.xc = (white + black)//2
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.delta = delta
        self.max_delta = max_delta
        self.i_error = FIFO(error)

        self.last_P = 0
        self.last_PI = 0
        self.last_PID = 0

    def instant_error(self):
        detect_light = self.color_sensor.detect_light()
        error = detect_light - self.xc
        self.i_error.push(error)
        return error

    def get_instant_error(self, n=1):
        if len(self.i_error) >= n:
            return self.i_error.peek(-n)
        else:
            return 0

    def bangbang_angle(self):
        error = self.instant_error()
        if error > 0:
            return self.delta
        elif error < 0:
            return -self.delta
        else:
            return 0

    def calc_P(self):
        error = self.instant_error()
        P = self.Kp * error
        self.last_P = P
        return P

    def calc_PI(self):
        P = self.calc_P()
        error = self.get_instant_error()
        error_sum = sum(self.i_error.data)

        delta = self.Ki * self.dt * error_sum 

        if delta > self.max_delta:
            delta = self.max_delta
        elif delta < -self.max_delta:
            delta = -self.max_delta

        delta = delta + P
        self.last_PI = delta
        return delta


    def calc_PID(self):
        PI = self.calc_PI()
        
        error = self.get_instant_error()
        error_prev = self.get_instant_error(2)

        derivative = (error - error_prev) / self.dt
        D = self.Kd * derivative

        delta = PI + D

        # delta = max(-self.max_delta, min(self.max_delta, delta))
        self.last_PID = delta
        return delta


    def get_PID_dict(self):
        return {
            "P": self.last_P,
            "PI": self.last_PI,
            "PID": self.last_PID,
        }
