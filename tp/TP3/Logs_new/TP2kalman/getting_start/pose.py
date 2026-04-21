#!/usr/bin/env pybricks-micropython
import math
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port

class Pose:
    def __init__(self, speed):
        self.gyro = GyroSensor(Port.S4)
        # Paramètres PID
        self.drive_speed = speed     # mm/s
        self.turn_rate = 0         # deg/s

        # Historique
        self.history = []

        # Position initiale (commune aux 3 méthodes)
        self.x_pid, self.y_pid = 0, 0
        self.x_state, self.y_state = 0, 0
        self.x_gyro, self.y_gyro = 0, 0

        # Orientation initiale
        self.theta_pid = 0
        self.theta_state = 0
        self.theta_gyro = 0
        
        # State
        self.state_distance = 0
        self.state_speed = 0
        self.state_angle = 0
        self.state_rot_speed = 0

    def update(self, turn_rate, state):
        dt = 0.1
        self.turn_rate = turn_rate
        
        # Mise à jour de la position via PID (cinématique)
        self.theta_pid += math.radians(self.turn_rate) * dt
        self.x_pid += self.drive_speed * dt * math.cos(self.theta_pid)
        self.y_pid += self.drive_speed * dt * math.sin(self.theta_pid)

        # --- Lecture via state() ---
        self.state_distance, self.state_speed, self.state_angle, self.state_rot_speed = state
        self.theta_state = math.radians(self.state_angle)
        self.x_state += self.state_speed * dt * math.cos(self.theta_state)
        self.y_state += self.state_speed * dt * math.sin(self.theta_state)

        # --- Lecture via gyroscope ---()
        self.theta_gyro = math.radians(self.gyro.angle())
        self.x_gyro += self.drive_speed * dt * math.cos(self.theta_gyro)
        self.y_gyro += self.drive_speed * dt * math.sin(self.theta_gyro)
        
