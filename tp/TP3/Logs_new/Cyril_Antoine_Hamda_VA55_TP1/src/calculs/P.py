class PController:

    def __init__(self, kp, setpoint):
        # Initialisation du contrôleur P.
        self.kp = kp
        self.setpoint = setpoint

    def compute(self, value):
        # Calcule la correction P.
        error = self.setpoint - value
        return self.kp * error
