from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port

class ColorSensorWrapper:

    def __init__(self, port):
        # Initialisation du capteur de couleur
        self.sensor = ColorSensor(port)

    def get_color(self):
        # Retourne la couleur détectée (nom).
        return self.sensor.color()

    def get_reflection(self):
        # Retourne la luminosité (0-100%).
        return self.sensor.reflection()

    def is_on_black(self, threshold=20):
        # Détecte si le capteur est sur du noir (luminosité faible).
        return self.get_reflection() < threshold

    def is_on_white(self, threshold=80):
        # Détecte si le capteur est sur du blanc (luminosité élevée).
        return self.get_reflection() > threshold
