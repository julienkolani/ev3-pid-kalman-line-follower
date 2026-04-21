class PIController:

    def __init__(self, kp, ki, setpoint):
        # Initialisation du contrôleur PI.
        self.kp = kp
        self.ki = ki
        self.setpoint = setpoint
        self.errors = []

    def compute(self, value):
        # Calcule la correction PI.
        error = self.setpoint - value
        self.errors.append(error)
        integral = sum(self.errors)
        return self.kp * error + self.ki * integral