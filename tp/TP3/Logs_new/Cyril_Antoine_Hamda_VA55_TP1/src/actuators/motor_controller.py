#!/usr/bin/env pybricks-micropython
"""
Module pour contrôler les moteurs du robot EV3.
"""

from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase

class MotorController:

    def __init__(self, left_port, right_port, wheel_diameter, axle_track):
        # Initialisation des moteurs sur les ports spécifiés
        self.left_motor = Motor(left_port)
        self.right_motor = Motor(right_port)

        # DriveBase avec paramètres configurables
        self.drive_base = DriveBase(
            self.left_motor, 
            self.right_motor, 
            wheel_diameter=wheel_diameter,  # ← PARAMÈTRE
            axle_track=axle_track          # ← PARAMÈTRE
        )

    def stop(self, stop_type=Stop.BRAKE):
        # Arrête le DriveBase
        self.drive_base.stop()

    def reset(self):
        # Réinitialise le DriveBase
        self.drive_base.reset()

    def get_status(self):
        # Retourne l'état actuel du DriveBase
        try:
            distance, speed, angle, turn_rate = self.drive_base.state()
            return {
                "distance": distance,
                "speed": speed,
                "angle": angle,
                "turn_rate": turn_rate,
                "left_angle": self.left_motor.angle(),
                "right_angle": self.right_motor.angle()
            }
        except:
            return {
                "left_angle": self.left_motor.angle(),
                "right_angle": self.right_motor.angle()
            }