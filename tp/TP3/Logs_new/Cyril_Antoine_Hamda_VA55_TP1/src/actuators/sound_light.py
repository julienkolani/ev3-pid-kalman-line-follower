from pybricks.hubs import EV3Brick
from pybricks.parameters import Color
from pybricks.tools import wait

class SoundLight:

    def __init__(self):
        #Initialiser le brique EV3
        self.ev3 = EV3Brick()

    def beep(self):
        # Joue un bip sonore
        self.ev3.speaker.beep()

    def play_tone(self, frequency, duration):
        # Joue un son à une fréquence et une durée spécifiées.
        self.ev3.speaker.beep(frequency=frequency, duration=duration)
    
    def set_leds(self, color=Color.GREEN):
        # Allume la LED de la couleur spécifiée
        self.ev3.light.on(color)
    
    def flash_leds(self, color=Color.RED, times=3, interval=200):
        # Flash les LEDs
        for _ in range(times):
            self.ev3.light.on(color)
            wait(interval)
            self.ev3.light.off()
            wait(interval)