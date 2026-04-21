from pybricks.hubs import EV3Brick

class LCDDisplay:

    def __init__(self):
        # Initialise la brique EV3
        self.ev3 = EV3Brick()

    def show_status(self, status_dict):
        # Affiche les informations sur l'écran LCD
        self.ev3.screen.clear()
        lines = []
        for key, value in status_dict.items():
            lines.append(str(key) + ": " + str(value))
        for i, line in enumerate(lines):
            self.ev3.screen.draw_text(10, 20 + i*20, line)