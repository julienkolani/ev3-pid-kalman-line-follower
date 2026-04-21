class PIDController:

    def __init__(self, *, kp, ki, kd, setpoint):
        # Initialisation du contrôleur PID
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.errors = []
        self.last_error = 0

    def compute(self, value):
        # Calcul de la sortie du contrôleur PID
        error = self.setpoint - value
        self.errors.append(error)
        integral = sum(self.errors)
        derivative = error - self.last_error
        self.last_error = error
        return self.kp * error + self.ki * integral + self.kd * derivative