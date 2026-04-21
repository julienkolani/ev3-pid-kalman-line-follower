from pybricks.ev3devices import UltrasonicSensor
from pybricks.parameters import Port

class DistanceSensorWrapper:

    def __init__(self, port):
        # Initialisation du capteur de distance
        self.sensor = UltrasonicSensor(port)

    def get_distance(self):
        # Retourne la distance mesurée en millimètres.
        return self.sensor.distance()

    def is_too_close(self, min_distance=100):
        # Détecte si un obstacle est trop proche (en mm).
        return self.get_distance() < min_distance
